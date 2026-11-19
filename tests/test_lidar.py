import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.lidar import Packet, PacketParser, Point, ScanAssembler, crc8


def make_frame(start=1000, end=1880, distance=1000, intensity=100):
    body = struct.pack("<BBHH", 0x54, 0x2C, 3600, start)
    body += struct.pack("<HB", distance, intensity) * 12
    body += struct.pack("<HH", end, 1234)
    return body + bytes([crc8(body)])


class ParserTests(unittest.TestCase):
    def test_crc_known_polynomial_table_entries(self):
        self.assertEqual(crc8(b"\x00"), 0)
        self.assertEqual(crc8(b"\x01"), 0x4D)
        self.assertEqual(crc8(b"\x02"), 0x9A)

    def test_partial_frame_with_noise(self):
        parser = PacketParser()
        frame = make_frame()
        self.assertFalse(parser.feed(b"noise" + frame[:10]))
        result = parser.feed(frame[10:])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].speed_hz, 10)
        self.assertEqual(result[0].points[0].distance, 1)
        self.assertAlmostEqual(result[0].points[-1].clockwise_degrees, 18.8)

    def test_crc_recovery(self):
        parser = PacketParser()
        damaged = bytearray(make_frame())
        damaged[8] ^= 1
        self.assertEqual(len(parser.feed(damaged + make_frame())), 1)
        self.assertGreater(parser.bad_packets, 0)

    def test_wrap_interpolation(self):
        packet = PacketParser().feed(make_frame(start=35500, end=380))[0]
        self.assertAlmostEqual(packet.points[0].clockwise_degrees, 355)
        self.assertAlmostEqual(packet.points[-1].clockwise_degrees, 3.8)

    def test_low_confidence_is_unknown(self):
        packet = PacketParser().feed(make_frame(intensity=15))[0]
        self.assertTrue(math.isnan(packet.points[0].distance))

    def test_zero_measurement_is_unknown(self):
        packet = PacketParser().feed(make_frame(distance=0))[0]
        self.assertTrue(math.isnan(packet.points[0].distance))

    def test_reject_impossible_sweep(self):
        self.assertFalse(PacketParser().feed(make_frame(start=100, end=9000)))

    def test_split_header_retained(self):
        parser = PacketParser()
        parser.feed(b"junk\x54")
        self.assertEqual(len(parser.feed(make_frame()[1:])), 1)


class AssemblerTests(unittest.TestCase):
    def test_first_partial_turn_discarded_and_clockwise_converted(self):
        assembler = ScanAssembler()
        self.assertFalse(assembler.add(Packet(10, 0, (Point(350, 1, 100),)), 0))
        self.assertFalse(assembler.add(Packet(10, 1, (Point(1, 1, 100), Point(90, 2, 100))), 0.01))
        assembler.add(Packet(10, 2, (Point(350, 1, 100),)), 0.09)
        scans = assembler.add(Packet(10, 3, (Point(1, 1, 100),)), 0.11)
        self.assertEqual(len(scans), 1)
        self.assertEqual(scans[0].ranges[270], 2)
        self.assertAlmostEqual(scans[0].duration, 0.1)

    def test_gap_discards_scan(self):
        assembler = ScanAssembler()
        assembler.add(Packet(10, 0, (Point(350, 1, 100),)), 0)
        assembler.add(Packet(10, 1, (Point(1, 1, 100),)), 0.01)
        assembler.add(Packet(10, 2, (Point(350, 1, 100),)), 0.5)
        self.assertFalse(assembler.add(Packet(10, 3, (Point(1, 1, 100),)), 0.51))


if __name__ == "__main__":
    unittest.main()
