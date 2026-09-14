"""Parser Micolink protokolu MTF-01P (MICOLINK_MSG_ID_RANGE_SENSOR = 0x51).

Payload (little-endian), zdroj: micoair.com/docs/decoding-micolink-messages-from-mtf-01
  0-3   time_ms      uint32  časové razítko senzoru [ms]
  4-7   distance     uint32  vzdálenost [mm], 0 = nedostupná
  8     strength     uint8   síla signálu dálkoměru
  9     precision    uint8   přesnost dálkoměru
  10    dis_status   uint8   status dálkoměru (0 = OK)
  11    reserved
  12-13 flow_vel_x   int16   optical flow rychlost X, jednotka cm/s @ 1m výšky
  14-15 flow_vel_y   int16   optical flow rychlost Y, jednotka cm/s @ 1m výšky
  16    flow_quality uint8   kvalita flow (0-255, vyšší = lepší)
  17    flow_status  uint8   status flow (0 = OK)

Skutečná rychlost = flow_vel * výška[m]  (proto je nutná kompenzace výškou).
"""
import struct
import time
from dataclasses import dataclass

MSG_ID_RANGE_SENSOR = 0x51


@dataclass
class RawFrame:
    host_time: float
    time_ms: int
    distance_mm: int
    strength: int
    precision: int
    dis_status: int
    flow_vel_x: int
    flow_vel_y: int
    flow_quality: int
    flow_status: int

    @property
    def distance_cm(self) -> float:
        return self.distance_mm / 10.0


def decode_payload(payload: bytes, host_time: float) -> RawFrame:
    time_ms, distance = struct.unpack('<II', payload[0:8])
    strength, precision, dis_status = payload[8], payload[9], payload[10]
    flow_vel_x, flow_vel_y = struct.unpack('<hh', payload[12:16])
    flow_quality, flow_status = payload[16], payload[17]
    return RawFrame(host_time, time_ms, distance, strength, precision, dis_status,
                     flow_vel_x, flow_vel_y, flow_quality, flow_status)


class MicolinkParser:
    """Stavový automat pro rozbalení Micolink rámců z proudu bajtů."""

    def __init__(self):
        self._state = 0
        self._packet = bytearray()
        self._payload_len = 0
        self.last_raw_packet: bytes = b''

    def feed(self, byte: int):
        """Vrátí RawFrame, jakmile je kompletní a validní rámec RANGE_SENSOR, jinak None."""
        s = self._state
        if s == 0:
            if byte == 0xEF:
                self._packet = bytearray([byte])
                self._state = 1
            return None
        self._packet.append(byte)
        if s in (1, 2, 3, 4):
            self._state = s + 1
        elif s == 5:
            self._payload_len = byte
            self._state = 6 if self._payload_len > 0 else 7
        elif s == 6:
            if len(self._packet) - 6 == self._payload_len:
                self._state = 7
        elif s == 7:
            self._state = 0
            checksum = sum(self._packet[:-1]) & 0xFF
            if checksum == byte and self._packet[3] == MSG_ID_RANGE_SENSOR and self._payload_len >= 18:
                self.last_raw_packet = bytes(self._packet)
                payload = self._packet[6:-1]
                return decode_payload(payload, time.time())
        return None
