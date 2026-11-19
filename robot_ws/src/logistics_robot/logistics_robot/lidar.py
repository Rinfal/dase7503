import math
import struct
from dataclasses import dataclass


def crc8(data):
    remainder = 0
    for byte in data:
        remainder ^= byte
        for _ in range(8):
            remainder = ((remainder << 1) ^ (0x4D if remainder & 0x80 else 0)) & 0xFF
    return remainder


@dataclass(frozen=True)
class Point:
    clockwise_degrees: float
    distance: float
    intensity: int


@dataclass(frozen=True)
class Packet:
    speed_hz: float
    timestamp_ms: int
    points: tuple


class PacketParser:
    def __init__(self):
        self.buffer = bytearray()
        self.valid_packets = 0
        self.bad_packets = 0

    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > 65536:
            del self.buffer[:-4096]
            self.bad_packets += 1
        packets = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(b"\x54\x2c")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x54 else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 47:
                break
            frame = bytes(self.buffer[:47])
            if crc8(frame[:-1]) != frame[-1]:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            speed, first_angle = struct.unpack_from("<HH", frame, 2)
            last_angle, timestamp = struct.unpack_from("<HH", frame, 42)
            sweep = (last_angle - first_angle) % 36000
            if (not 1800 <= speed <= 7200 or first_angle >= 36000 or last_angle >= 36000
                    or sweep > 6000 or timestamp >= 30000):
                self.bad_packets += 1
                del self.buffer[:47]
                continue
            points = []
            for index in range(12):
                distance_mm, intensity = struct.unpack_from("<HB", frame, 6 + index * 3)
                distance = distance_mm / 1000.0
                if intensity < 16 or not 0.03 <= distance <= 12:
                    distance = math.nan
                angle = ((first_angle + sweep * index / 11) / 100.0) % 360.0
                points.append(Point(angle, distance, intensity))
            packets.append(Packet(speed / 360.0, timestamp, tuple(points)))
            self.valid_packets += 1
            del self.buffer[:47]
        return packets


@dataclass(frozen=True)
class Scan:
    started: float
    duration: float
    ranges: tuple
    intensities: tuple


class ScanAssembler:
    def __init__(self, bins=360):
        self.bins = bins
        self.last_angle = None
        self.started = None
        self.last_packet_time = None
        self._clear()

    def _clear(self):
        self.ranges = [math.nan] * self.bins
        self.intensities = [0.0] * self.bins

    def add(self, packet, now):
        scans = []
        if self.last_packet_time is not None and now - self.last_packet_time > 0.15:
            self.started = None
            self.last_angle = None
            self._clear()
        self.last_packet_time = now
        for point in packet.points:
            wrapped = (self.last_angle is not None and self.last_angle > 300
                       and point.clockwise_degrees < 60)
            if wrapped:
                if self.started is not None and 0.04 <= now - self.started <= 0.25:
                    scans.append(Scan(self.started, now - self.started,
                                      tuple(self.ranges), tuple(self.intensities)))
                self.started = now
                self._clear()
            if self.started is not None:
                index = (-round(point.clockwise_degrees * self.bins / 360)) % self.bins
                previous = self.ranges[index]
                if math.isfinite(point.distance) and (not math.isfinite(previous) or point.distance < previous):
                    self.ranges[index] = point.distance
                    self.intensities[index] = float(point.intensity)
            self.last_angle = point.clockwise_degrees
        return scans
