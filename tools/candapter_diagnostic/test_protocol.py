import time
import unittest

from candapter import parse_candapter_frame
from protocol import (
    CANFrame,
    decode_frame,
    decoded_time_series,
    fault_status_rows,
    pack_soc_trend,
)


class ProtocolTests(unittest.TestCase):
    def test_temperature_decode(self) -> None:
        frame = CANFrame(0x108, bytes.fromhex("00 1B 00 1D"), time.time())
        decoded = decode_frame(frame)
        self.assertEqual(decoded["fields"]["Lowest temperature"], 27)
        self.assertEqual(decoded["fields"]["Highest temperature"], 29)

    def test_electrical_decode(self) -> None:
        frame = CANFrame(
            0x109,
            bytes.fromhex("85 CF 83 3F 7F FF"),
            time.time(),
        )
        decoded = decode_frame(frame)
        self.assertAlmostEqual(
            decoded["fields"]["Highest cell voltage"],
            3.4255,
        )
        self.assertAlmostEqual(
            decoded["fields"]["Lowest cell voltage"],
            3.3599,
        )
        self.assertAlmostEqual(decoded["fields"]["Pack current"], 0.1)

    def test_fault_status_decode(self) -> None:
        frame = CANFrame(0x505, b"\x01\x00", time.time())
        decoded = decode_frame(frame)
        self.assertTrue(decoded["fields"]["Fault active"])

    def test_fault_parameter_decode(self) -> None:
        frame = CANFrame(
            0x505,
            b"\x01\x01",
            time.time(),
        )
        decoded = decode_frame(frame)
        self.assertEqual(
            decoded["fields"]["Violated parameter"],
            "Cell overvoltage",
        )

    def test_fault_status_formats_reach_overview(self) -> None:
        for payload in (b"\x00", b"\x01", b"\x00\x00", b"\x01\x02"):
            with self.subTest(payload=payload):
                frame = CANFrame(0x505, payload, time.time())
                decoded = decode_frame(frame)
                active = bool(payload[0] & 1)
                self.assertEqual(decoded["error"], "")
                self.assertEqual(decoded["fields"]["Fault active"], active)
                if len(payload) == 1:
                    self.assertEqual(
                        decoded["fields"]["Violated parameter"], "Not reported"
                    )
                row = next(
                    row for row in fault_status_rows({0x505: frame})
                    if row["Fault"] == "Powertrain combined fault"
                )
                self.assertEqual(row["Status"], "FAULT" if active else "OK")

    def test_invalid_fault_status_is_unknown(self) -> None:
        for payload in (b"", b"\x01\x02\x03"):
            with self.subTest(payload=payload):
                frame = CANFrame(0x505, payload, time.time())
                self.assertTrue(decode_frame(frame)["error"])
                row = next(
                    row for row in fault_status_rows({0x505: frame})
                    if row["Fault"] == "Powertrain combined fault"
                )
                self.assertEqual(row["Status"], "UNKNOWN")

    def test_candapter_standard_frame_parse(self) -> None:
        frame = parse_candapter_frame("T109685CF833F7FFF")
        self.assertIsNotNone(frame)
        assert frame is not None
        self.assertEqual(frame.arbitration_id, 0x109)
        self.assertEqual(frame.data, bytes.fromhex("85 CF 83 3F 7F FF"))

    def test_broadcast_soc(self) -> None:
        frame = CANFrame(
            0x101,
            bytes.fromhex("00 00 00 00 64 00 00 00"),
            time.time(),
        )
        decoded = decode_frame(frame)
        self.assertEqual(decoded["fields"]["State of charge"], 50.0)

    def test_pack_soc_trends(self) -> None:
        now = time.time()

        def soc_frame(raw_soc: int, timestamp: float) -> CANFrame:
            return CANFrame(
                0x101,
                bytes([0, 0, 0, 0, raw_soc, 0, 0, 0]),
                timestamp,
            )

        cases = {
            "charging": (
                [soc_frame(100, now), soc_frame(101, now + 10)],
                "CHARGING",
                0.5,
            ),
            "discharging": (
                [soc_frame(101, now), soc_frame(100, now + 10)],
                "DISCHARGING",
                -0.5,
            ),
            "holding": (
                [soc_frame(100, now), soc_frame(100, now + 10)],
                "HOLDING",
                0.0,
            ),
        }
        for name, (frames, expected_state, expected_delta) in cases.items():
            with self.subTest(name=name):
                trend = pack_soc_trend(frames)
                self.assertEqual(trend["state"], expected_state)
                self.assertEqual(trend["delta_percent"], expected_delta)

    def test_pack_soc_trend_requires_two_samples(self) -> None:
        frame = CANFrame(
            0x101,
            bytes.fromhex("00 00 00 00 64 00 00 00"),
            time.time(),
        )
        self.assertEqual(
            pack_soc_trend([frame]),
            {"state": "UNKNOWN", "delta_percent": None},
        )

    def test_decoded_time_series_selects_and_sorts_signal(self) -> None:
        now = time.time()
        frames = [
            CANFrame(
                0x101,
                bytes.fromhex("00 00 00 00 66 00 00 00"),
                now + 2,
            ),
            CANFrame(0x108, bytes.fromhex("00 1B 00 1D"), now + 1),
            CANFrame(
                0x101,
                bytes.fromhex("00 00 00 00 64 00 00 00"),
                now,
            ),
        ]

        self.assertEqual(
            decoded_time_series(frames, "pack_soc"),
            [
                {"timestamp": now, "value": 50.0},
                {"timestamp": now + 2, "value": 51.0},
            ],
        )

    def test_decoded_time_series_rejects_unknown_signal(self) -> None:
        with self.assertRaises(ValueError):
            decoded_time_series([], "not_a_signal")

    def test_normal_frames_have_no_value_faults(self) -> None:
        now = time.time()
        latest = {
            0x108: CANFrame(0x108, bytes.fromhex("00 1B 00 1D"), now),
            0x109: CANFrame(
                0x109,
                bytes.fromhex("85 CF 83 3F 7F FF"),
                now,
            ),
            0x505: CANFrame(0x505, b"\x00", now),
        }
        rows = fault_status_rows(latest)
        self.assertTrue(all(row["Status"] == "OK" for row in rows))


if __name__ == "__main__":
    unittest.main()
