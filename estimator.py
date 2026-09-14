"""Filtrace a odhad rychlosti/výšky z MTF-01P rámců.

Klíčové principy (proč "skáče" a jak to řešit):
  1. Senzor posílá i nekvalitní vzorky (nízký flow_quality, chybový dis_status/
     flow_status) - ty se nesmí pouštět do filtru vůbec, jinak je i silný EMA
     rozhodí. Špatný vzorek => hold (podrž poslední dobrou hodnotu).
  2. I mezi kvalitními vzorky se objeví ojedinělé odlehlé hodnoty (spike) -
     řeší Hampel filtr (medián + MAD), který na rozdíl od prostého mediánu
     propustí skutečnou (trvalou) změnu, ale odstraní jednorázový skok.
  3. AGV většinu času stojí/jede rovně a v klidu musí být rychlost přesně 0 -
     detekce klidu (ZUPT) jednak "přiškrtí" výstup na 0 (žádný drift v
     integrátoru trajektorie), jednak pomalu doučuje nulový offset (bias)
     flow senzoru, který se mění s teplotou/osvětlením.
  4. dt pro integraci trajektorie se počítá z časového razítka senzoru
     (time_ms), ne z hodinek hostitele - eliminuje jitter OS/UDP přenosu.
"""
import numpy as np
from collections import deque
from dataclasses import dataclass

from mtf01_link import RawFrame


@dataclass
class Config:
    target_height_cm: float = 15.0
    calib_samples: int = 300          # ~3 s při 100 Hz

    quality_min: int = 30             # flow_quality [0-255], nastav v kalibr. nástroji
    # Výrobce nezveřejnil význam hodnot dis_status/flow_status a reálný senzor
    # trvale hlásí hodnotu 1 (ne 0, jak by se čekalo u typické "0=OK" konvence) -
    # proto se defaultně na status NEFILTRUJE (None = přijmout cokoliv). Hodnoty
    # se zobrazují v calibration_tool.py - pokud se při reálné chybě (zakrytí
    # senzoru, mimo dosah) objeví jiná hodnota, nastav sem konkrétní tuple
    # "dobrých" hodnot, např. (0, 1).
    dis_status_ok: tuple | None = None
    flow_status_ok: tuple | None = None

    median_window: int = 7
    hampel_window: int = 11
    hampel_k: float = 5.0             # kolik MAD = odlehlá hodnota

    ema_alpha_vel: float = 0.35
    ema_alpha_dist: float = 0.35          # pro zobrazení výšky (rychle sleduje i hrboly)
    ema_alpha_height_vel: float = 0.1     # vyhlazení kandidátní výšky pro zámek níže -
                                           # jen aby single-sample výkyv nespustil odpočet

    # Reálně naměřeno (viz docs/MTF01P_manual.md): výška u ležícího senzoru
    # nekolísá jako běžný šum, ale skáče mezi několika hladinami po dobu
    # řádově 1-4 s (typicky ToF senzor mění vnitřní expozici/gain). Žádná
    # rozumně rychlá EMA to nevyhladí, aniž by zavlekla stejně velké
    # zpoždění. Protože výška u AGV je prakticky konstantní a mění se jen
    # při reálném najetí na překážku, řeší se to zámkem (hystereze):
    # výška použitá pro škálování rychlosti (height_lock_cm) se drží pevně
    # a mění se, jen když odchylka od ní trvá souvisle déle než
    # height_lock_confirm_s - tehdy se vyhodnotí jako reálná změna výšky.
    height_lock_thresh_cm: float = 2.0
    height_lock_confirm_s: float = 0.3

    stationary_window: int = 20
    stationary_std_thresh: float = 2.5   # cm/s - pod touto směrodatnou odchylkou = klid
    bias_tracking_alpha: float = 0.01    # rychlost doučování nulového bodu v klidu
    bias_gate_raw: float = 8.0           # [raw flow jednotky] okamžitá pojistka proti
                                          # doučování bias při rozjezdu (viz update())


@dataclass
class Estimate:
    t: float
    vx: float = 0.0
    vy: float = 0.0
    height_cm: float = 0.0
    height_lock_cm: float = 0.0
    flow_ok: bool = False
    dist_ok: bool = False
    stationary: bool = False
    flow_quality: int = 0
    calibrating: bool = True
    calib_progress: float = 0.0


def _hampel_filter(buf: deque, value: float, k: float):
    """Vrátí (accepted_value, is_outlier). buf obsahuje historii SYROVÝCH hodnot.

    Do historie se vždy ukládá syrová hodnota (ne přefiltrovaná) - jinak by
    filtr při trvalé změně (skutečný rozjezd, ne šumový skok) sám sebe navždy
    oslepil: každý nový vzorek by vypadal jako odlehlý oproti staré historii,
    která by se bez syrových dat nikdy neposunula k nové realitě.
    """
    if len(buf) < 5:
        buf.append(value)
        return value, False
    arr = np.fromiter(buf, dtype=float)
    med = float(np.median(arr))
    mad = float(np.median(np.abs(arr - med))) * 1.4826 + 1e-6
    is_outlier = abs(value - med) > k * mad
    buf.append(value)
    return (med, True) if is_outlier else (value, False)


class FlowEstimator:
    def __init__(self, cfg: Config, calibration: dict | None = None):
        self.cfg = cfg
        self.dist_offset = 0.0
        self.bias_x = 0.0
        self.bias_y = 0.0
        if calibration:
            self.dist_offset = calibration.get('dist_offset', 0.0)
            self.bias_x = calibration.get('bias_x', 0.0)
            self.bias_y = calibration.get('bias_y', 0.0)

        self.calibrating = calibration is None
        self._calib_n = 0
        self._calib_sum_dist = 0.0
        self._calib_sum_x = 0.0
        self._calib_sum_y = 0.0

        self._dist_hist = deque(maxlen=cfg.hampel_window)
        self._vx_hist = deque(maxlen=cfg.hampel_window)
        self._vy_hist = deque(maxlen=cfg.hampel_window)

        self._med_dist = deque(maxlen=cfg.median_window)
        self._med_vx = deque(maxlen=cfg.median_window)
        self._med_vy = deque(maxlen=cfg.median_window)

        self._stat_vx = deque(maxlen=cfg.stationary_window)
        self._stat_vy = deque(maxlen=cfg.stationary_window)

        self.ema_dist = None
        self.ema_dist_slow = None
        self.height_lock_cm = None
        self._height_dev_time = 0.0
        self.ema_vx = 0.0
        self.ema_vy = 0.0

        self._last_sensor_ms = None
        self.pos_x = 0.0
        self.pos_y = 0.0

        self.n_frames = 0
        self.n_dropped_quality = 0
        self.n_outliers = 0

    # ------------------------------------------------------------------
    def reset_calibration(self):
        self.calibrating = True
        self._calib_n = 0
        self._calib_sum_dist = 0.0
        self._calib_sum_x = 0.0
        self._calib_sum_y = 0.0

    def reset_trajectory(self):
        self.pos_x = 0.0
        self.pos_y = 0.0

    def export_calibration(self) -> dict:
        return {'dist_offset': self.dist_offset, 'bias_x': self.bias_x, 'bias_y': self.bias_y}

    # ------------------------------------------------------------------
    def update(self, f: RawFrame) -> Estimate:
        self.n_frames += 1
        dist_ok = (self.cfg.dis_status_ok is None or f.dis_status in self.cfg.dis_status_ok) and f.distance_mm > 0
        flow_ok = (self.cfg.flow_status_ok is None or f.flow_status in self.cfg.flow_status_ok) \
            and f.flow_quality >= self.cfg.quality_min

        if self.calibrating:
            if dist_ok and flow_ok:
                self._calib_sum_dist += f.distance_cm
                self._calib_sum_x += f.flow_vel_x
                self._calib_sum_y += f.flow_vel_y
                self._calib_n += 1
            if self._calib_n >= self.cfg.calib_samples:
                avg_dist = self._calib_sum_dist / self._calib_n
                self.dist_offset = self.cfg.target_height_cm - avg_dist
                self.bias_x = self._calib_sum_x / self._calib_n
                self.bias_y = self._calib_sum_y / self._calib_n
                self.calibrating = False
                self.ema_dist = None
                self.ema_dist_slow = None
                self.height_lock_cm = avg_dist
                self._height_dev_time = 0.0
            return Estimate(f.host_time, calibrating=True,
                             calib_progress=self._calib_n / self.cfg.calib_samples,
                             flow_quality=f.flow_quality, dist_ok=dist_ok, flow_ok=flow_ok)

        # ---- dt (ze senzorového času - eliminuje jitter OS/UDP přenosu) ----
        if self._last_sensor_ms is None:
            dt = 0.01
        else:
            dt_ms = f.time_ms - self._last_sensor_ms
            if dt_ms < 0:
                dt_ms += 1 << 32
            dt = dt_ms / 1000.0
            if dt <= 0 or dt > 0.5:
                dt = 0.01
        self._last_sensor_ms = f.time_ms

        # ---- výška ----
        if dist_ok:
            val, outlier = _hampel_filter(self._dist_hist, f.distance_cm, self.cfg.hampel_k)
            if outlier:
                self.n_outliers += 1
            self._med_dist.append(val)
            med_dist = float(np.median(self._med_dist))
            self.ema_dist = med_dist if self.ema_dist is None else (
                self.cfg.ema_alpha_dist * med_dist + (1 - self.cfg.ema_alpha_dist) * self.ema_dist)
            self.ema_dist_slow = med_dist if self.ema_dist_slow is None else (
                self.cfg.ema_alpha_height_vel * med_dist + (1 - self.cfg.ema_alpha_height_vel) * self.ema_dist_slow)

            if self.height_lock_cm is None:
                self.height_lock_cm = self.ema_dist_slow
            if abs(self.ema_dist_slow - self.height_lock_cm) > self.cfg.height_lock_thresh_cm:
                self._height_dev_time += dt
                if self._height_dev_time >= self.cfg.height_lock_confirm_s:
                    self.height_lock_cm = self.ema_dist_slow
                    self._height_dev_time = 0.0
            else:
                self._height_dev_time = 0.0

        # height_cm (zobrazovaná/diagnostická) reaguje rychle, aby šlo v grafu
        # vidět skutečné kolísání senzoru. height_m pro škálování rychlosti
        # vychází ze "zamčené" výšky (height_lock_cm) - viz Config a komentář
        # u height_lock_thresh_cm/height_lock_confirm_s.
        height_cm = (self.ema_dist if self.ema_dist is not None else self.cfg.target_height_cm) + self.dist_offset
        height_m = ((self.height_lock_cm if self.height_lock_cm is not None else self.cfg.target_height_cm)
                    + self.dist_offset) / 100.0

        # ---- rychlost (flow) ----
        if not flow_ok:
            self.n_dropped_quality += 1
            real_vx, real_vy = self.ema_vx * height_m, self.ema_vy * height_m
        else:
            corr_x = f.flow_vel_x - self.bias_x
            corr_y = f.flow_vel_y - self.bias_y

            vx_h, out_x = _hampel_filter(self._vx_hist, corr_x, self.cfg.hampel_k)
            vy_h, out_y = _hampel_filter(self._vy_hist, corr_y, self.cfg.hampel_k)
            if out_x or out_y:
                self.n_outliers += 1

            self._med_vx.append(vx_h)
            self._med_vy.append(vy_h)
            med_x = float(np.median(self._med_vx))
            med_y = float(np.median(self._med_vy))

            self.ema_vx = self.cfg.ema_alpha_vel * med_x + (1 - self.cfg.ema_alpha_vel) * self.ema_vx
            self.ema_vy = self.cfg.ema_alpha_vel * med_y + (1 - self.cfg.ema_alpha_vel) * self.ema_vy

            real_vx, real_vy = self.ema_vx * height_m, self.ema_vy * height_m

        # ---- detekce klidu (ZUPT) + doučování nulového bodu ----
        # Pozor: do okna jde kandidátní (nezaškrcnutá) hodnota a vnitřní EMA stav
        # (self.ema_vx/self.ema_vy) se NIKDY nenuluje - jen výstup pro tento snímek.
        # Kdyby se nulovala i vnitřní paměť filtru, vznikne past: jakmile filtr
        # jednou vyhodnotí klid, sám sebe udrží na nule navěky, protože vlastní
        # historie (kterou si sám nuloval) mu klid pořád potvrzuje.
        self._stat_vx.append(real_vx)
        self._stat_vy.append(real_vy)
        stationary = False
        if len(self._stat_vx) == self.cfg.stationary_window:
            # RMS (ne směr. odchylka!) - odchylka samotná by klidnou jízdu
            # konstantní rychlostí (nízký rozptyl, ale nenulová hodnota) vyhodnotila
            # mylně jako klid. RMS roste jak s rozptylem, tak s odchylkou od nuly.
            rms = float(np.sqrt(np.mean(np.square(self._stat_vx)) + np.mean(np.square(self._stat_vy))))
            if rms < self.cfg.stationary_std_thresh:
                stationary = True

        # Okno pro "stationary" je zprůměrované přes stationary_window vzorků,
        # takže po skutečném rozjezdu ještě pár desítek ms hlásí klid (lag).
        # Bez okamžité pojistky na aktuální (nezprůměrovaný) vzorek by se bias
        # v tu chvíli začal učit směrem k jízdní rychlosti = zkažená kalibrace
        # při každém rozjezdu. Adaptace bias smí běžet jen když i tento
        # konkrétní vzorek je blízko nule.
        if stationary and flow_ok and abs(corr_x) < self.cfg.bias_gate_raw and abs(corr_y) < self.cfg.bias_gate_raw:
            a = self.cfg.bias_tracking_alpha
            self.bias_x += a * (f.flow_vel_x - self.bias_x)
            self.bias_y += a * (f.flow_vel_y - self.bias_y)

        if stationary:
            real_vx = 0.0
            real_vy = 0.0

        # ---- integrace trajektorie (dt spočteno výše ze senzorového času) ----
        self.pos_x += real_vx * dt
        self.pos_y += real_vy * dt

        return Estimate(f.host_time, vx=real_vx, vy=real_vy, height_cm=height_cm,
                         height_lock_cm=height_m * 100.0,
                         flow_ok=flow_ok, dist_ok=dist_ok, stationary=stationary,
                         flow_quality=f.flow_quality, calibrating=False)
