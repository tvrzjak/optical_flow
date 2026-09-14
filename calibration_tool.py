"""Kalibrační a testovací nástroj pro MTF-01P.

Zdroj dat:
    --serial /dev/ttyAMA0          přímé připojení k senzoru (lokálně na RPi)
    --udp-raw 12346                příjem raw rámců z raw_forwarder.py (odkudkoli v síti)

Ovládání za běhu:
    Rekalibrovat   - nové měření nulového bodu (drž senzor nehybně)
    Reset trajektorie
    Uložit kalibraci -> calibration.json
    Slidery         - živé ladění filtru (quality_min, EMA alpha, Hampel k)
"""
import argparse
import os
import queue
import socket
import threading
import time

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from collections import deque

import serial

from mtf01_link import MicolinkParser, decode_payload
from estimator import Config, FlowEstimator
from calib_store import load_calibration, save_calibration

HIST = 600  # ~ posledních N vzorků v grafech
DEFAULT_CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.json')

MIN_VEL_SPAN = 5.0    # cm/s - minimální rozsah osy Y u grafu rychlosti (viz update())
MIN_HEIGHT_SPAN = 3.0  # cm - totéž pro výšku


def _enforce_min_span(ax, min_span: float):
    """Vynutí minimální rozsah osy Y, větší rozsah (reálný pohyb) nechá být.

    V klidu je vx/vy prakticky konstantní (řádově 1e-16 kvůli zaokrouhlení
    EMA) - prostý autoscale by se na tuto mikroskopickou škálu přiblížil a
    graf by opticky "poskakoval". Řešením je jen dolní mez rozsahu; horní mez
    není omezená (při reálném pohybu osa naroste) a po zklidnění se zase sama
    zmenší zpět na min_span, jakmile velké hodnoty vypadnou z zobrazovaného
    okna (žádný trvalý stav mezi snímky, počítá se to znovu při každém
    překreslení).
    """
    ymin, ymax = ax.get_ylim()
    span = ymax - ymin
    if span < min_span:
        mid = (ymax + ymin) / 2
        ax.set_ylim(mid - min_span / 2, mid + min_span / 2)


def serial_reader(port, baud, frame_q: queue.Queue, stop_evt: threading.Event):
    ser = serial.Serial(port, baud, timeout=0.05)
    parser = MicolinkParser()
    while not stop_evt.is_set():
        chunk = ser.read(64)
        for byte in chunk:
            frame = parser.feed(byte)
            if frame is not None:
                frame_q.put(frame)
    ser.close()


def udp_raw_reader(udp_port, frame_q: queue.Queue, stop_evt: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', udp_port))
    sock.settimeout(0.2)
    while not stop_evt.is_set():
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) < 19 or data[3] != 0x51:
            continue
        payload = data[6:-1]
        frame_q.put(decode_payload(payload, time.time()))
    sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--serial', metavar='PORT', help='např. /dev/ttyAMA0')
    src.add_argument('--udp-raw', metavar='PORT', type=int, help='poslouchej raw rámce z raw_forwarder.py')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--height', type=float, default=15.0)
    ap.add_argument('--calib-file', default=DEFAULT_CALIB_FILE)
    args = ap.parse_args()

    cfg = Config(target_height_cm=args.height)
    calibration = load_calibration(args.calib_file)
    est = FlowEstimator(cfg, calibration)

    frame_q: queue.Queue = queue.Queue(maxsize=5000)
    stop_evt = threading.Event()
    if args.serial:
        t = threading.Thread(target=serial_reader, args=(args.serial, args.baud, frame_q, stop_evt), daemon=True)
    else:
        t = threading.Thread(target=udp_raw_reader, args=(args.udp_raw, frame_q, stop_evt), daemon=True)
    t.start()

    # ---- data buffery pro grafy ----
    t_hist = deque(maxlen=HIST)
    vx_hist = deque(maxlen=HIST)
    vy_hist = deque(maxlen=HIST)
    h_hist = deque(maxlen=HIST)
    h_raw_hist = deque(maxlen=HIST)
    h_lock_hist = deque(maxlen=HIST)
    q_hist = deque(maxlen=HIST)
    stat_hist = deque(maxlen=HIST)
    t0 = None

    fig = plt.figure(figsize=(15, 9))
    fig.suptitle('MTF-01P – kalibrace a live test')
    gs = fig.add_gridspec(3, 2, width_ratios=[3, 1], height_ratios=[1, 1, 1],
                           left=0.07, right=0.98, top=0.93, bottom=0.30, hspace=0.4)
    ax_vel = fig.add_subplot(gs[0, 0])
    ax_h = fig.add_subplot(gs[1, 0], sharex=ax_vel)
    ax_q = fig.add_subplot(gs[2, 0], sharex=ax_vel)
    ax_traj = fig.add_subplot(gs[:, 1])
    ax_text = fig.add_axes([0.07, 0.205, 0.9, 0.06]); ax_text.axis('off')
    txt = ax_text.text(0, 1.0, '', family='monospace', fontsize=9, va='top')

    line_vx, = ax_vel.plot([], [], 'r-', lw=1.8, label='Vx [cm/s]')
    line_vy, = ax_vel.plot([], [], 'g-', lw=1.8, label='Vy [cm/s]')
    ax_vel.axhline(0, color='k', ls='--', lw=0.8)
    ax_vel.set_ylabel('rychlost [cm/s]'); ax_vel.legend(loc='upper right'); ax_vel.grid(alpha=0.3)

    line_h, = ax_h.plot([], [], 'c-', lw=0.8, alpha=0.5, label='výška raw (zobrazení)')
    line_hr, = ax_h.plot([], [], 'k:', lw=0.8, alpha=0.6, label='výška surová')
    line_hlock, = ax_h.plot([], [], 'b-', lw=2.0, label='výška zamčená (pro rychlost)')
    ax_h.set_ylabel('výška [cm]'); ax_h.legend(loc='upper right', fontsize=8); ax_h.grid(alpha=0.3)

    line_q, = ax_q.plot([], [], 'm-', lw=1.2, label='flow_quality')
    line_thr = ax_q.axhline(cfg.quality_min, color='k', ls='--', lw=0.8, label='práh')
    ax_q.set_ylabel('quality'); ax_q.set_xlabel('t [s]'); ax_q.legend(loc='upper right'); ax_q.grid(alpha=0.3)

    line_traj, = ax_traj.plot([], [], 'b-', lw=1.5)
    point_traj, = ax_traj.plot([], [], 'ro', ms=8)
    ax_traj.set_xlabel('Y [cm]'); ax_traj.set_ylabel('X [cm]'); ax_traj.grid(alpha=0.3); ax_traj.axis('equal')
    ax_traj.set_title('Trajektorie v souřadnicích senzoru\n(dead-reckoning, bez korekce natočení - viz IMU)')

    # ---- widgety ----
    s_q = Slider(fig.add_axes([0.12, 0.16, 0.3, 0.03]), 'quality_min', 0, 255, valinit=cfg.quality_min, valstep=1)
    s_ea_v = Slider(fig.add_axes([0.12, 0.11, 0.3, 0.03]), 'EMA alpha vel', 0.01, 1.0, valinit=cfg.ema_alpha_vel)
    s_ea_d = Slider(fig.add_axes([0.12, 0.06, 0.3, 0.03]), 'EMA alpha dist (zobrazení)', 0.01, 1.0, valinit=cfg.ema_alpha_dist)
    s_hlock = Slider(fig.add_axes([0.12, 0.01, 0.3, 0.03]), 'práh zámku výšky [cm]', 0.3, 6.0, valinit=cfg.height_lock_thresh_cm)
    s_hk = Slider(fig.add_axes([0.55, 0.16, 0.3, 0.03]), 'Hampel k', 1.0, 15.0, valinit=cfg.hampel_k)
    s_stat = Slider(fig.add_axes([0.55, 0.11, 0.3, 0.03]), 'práh klidu [cm/s]', 0.2, 10.0, valinit=cfg.stationary_std_thresh)

    def on_slider(_):
        cfg.quality_min = int(s_q.val)
        cfg.ema_alpha_vel = s_ea_v.val
        cfg.ema_alpha_dist = s_ea_d.val
        cfg.height_lock_thresh_cm = s_hlock.val
        cfg.hampel_k = s_hk.val
        cfg.stationary_std_thresh = s_stat.val
        line_thr.set_ydata([cfg.quality_min, cfg.quality_min])
    for s in (s_q, s_ea_v, s_ea_d, s_hlock, s_hk, s_stat):
        s.on_changed(on_slider)

    def on_recalibrate(_):
        est.reset_calibration()
    def on_reset_traj(_):
        est.reset_trajectory()
    def on_save(_):
        save_calibration(args.calib_file, est.export_calibration())
        print(f"Kalibrace uložena do {args.calib_file}")

    b_recal = Button(fig.add_axes([0.55, 0.06, 0.13, 0.035]), 'Rekalibrovat')
    b_recal.on_clicked(on_recalibrate)
    b_reset = Button(fig.add_axes([0.70, 0.06, 0.13, 0.035]), 'Reset trajektorie')
    b_reset.on_clicked(on_reset_traj)
    b_save = Button(fig.add_axes([0.85, 0.06, 0.13, 0.035]), 'Uložit kalibraci')
    b_save.on_clicked(on_save)

    s_hconfirm = Slider(fig.add_axes([0.55, 0.01, 0.3, 0.03]), 'potvrz. změny výšky [s]', 0.05, 2.0, valinit=cfg.height_lock_confirm_s)
    s_hconfirm.on_changed(lambda _: setattr(cfg, 'height_lock_confirm_s', s_hconfirm.val))

    traj_x_hist = deque(maxlen=5000)
    traj_y_hist = deque(maxlen=5000)

    def update(_frame):
        nonlocal t0
        drained = 0
        while drained < 500:
            try:
                frame = frame_q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            out = est.update(frame)
            if out.calibrating:
                txt.set_text(f"KALIBRACE {out.calib_progress*100:5.1f}% - drž senzor nehybně na ~{cfg.target_height_cm:.0f} cm")
                continue
            if t0 is None:
                t0 = out.t
            t_hist.append(out.t - t0)
            vx_hist.append(out.vx)
            vy_hist.append(out.vy)
            h_hist.append(out.height_cm)
            h_raw_hist.append(frame.distance_cm + est.dist_offset)
            h_lock_hist.append(out.height_lock_cm)
            q_hist.append(out.flow_quality)
            stat_hist.append(out.stationary)

        if len(t_hist) < 5:
            return ()

        tt = np.array(t_hist)
        line_vx.set_data(tt, vx_hist); line_vy.set_data(tt, vy_hist)
        line_h.set_data(tt, h_hist); line_hr.set_data(tt, h_raw_hist)
        line_hlock.set_data(tt, h_lock_hist)
        line_q.set_data(tt, q_hist)
        for ax in (ax_vel, ax_h, ax_q):
            ax.set_xlim(tt[0], tt[-1] if tt[-1] > tt[0] else tt[0] + 1)
        # set_ylim() v _enforce_min_span (níže) natrvalo vypne autoscale pro
        # danou osu (chování matplotlib) - protože se tu na rozdíl od ax.clear()
        # nikdy neresetuje, musí se před KAŽDÝM voláním autoscale_view ručně
        # zapnout znovu, jinak osa po prvním zásahu zamrzne a přestane
        # reagovat na větší data.
        ax_vel.set_autoscaley_on(True); ax_vel.relim(); ax_vel.autoscale_view(scalex=False)
        ax_h.set_autoscaley_on(True); ax_h.relim(); ax_h.autoscale_view(scalex=False)
        _enforce_min_span(ax_vel, MIN_VEL_SPAN)
        _enforce_min_span(ax_h, MIN_HEIGHT_SPAN)
        ax_q.set_ylim(0, 260)

        traj_x_hist.append(est.pos_x)
        traj_y_hist.append(est.pos_y)
        line_traj.set_data(traj_y_hist, traj_x_hist)
        point_traj.set_data([est.pos_y], [est.pos_x])
        ax_traj.relim(); ax_traj.autoscale_view()

        n = max(est.n_frames, 1)
        stat_now = stat_hist[-1] if stat_hist else False
        vx_arr = np.array(vx_hist)[-100:]
        vy_arr = np.array(vy_hist)[-100:]
        txt.set_text(
            f"vzorků: {n:6d} | zahozeno (kvalita): {est.n_dropped_quality/n*100:4.1f}% | "
            f"odlehlé: {est.n_outliers/n*100:4.1f}% | bias=({est.bias_x:6.1f},{est.bias_y:6.1f}) | "
            f"dist_offset={est.dist_offset:5.2f} cm | výška zamčená={out.height_lock_cm:5.1f} cm | "
            f"šum Vx/Vy (posl. 100): {np.std(vx_arr):.2f}/{np.std(vy_arr):.2f} cm/s | static={stat_now}\n"
            f"posl. rámec: strength={frame.strength} precision={frame.precision} "
            f"dis_status={frame.dis_status} flow_status={frame.flow_status}  "
            f"(sleduj, zda se status mění při zakrytí/vyjetí z dosahu senzoru)"
        )
        return ()

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

    try:
        plt.show()
    finally:
        stop_evt.set()
        t.join(timeout=1.0)


if __name__ == '__main__':
    main()
