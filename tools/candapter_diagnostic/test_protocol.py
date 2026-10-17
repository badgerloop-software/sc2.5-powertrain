import struct
import time
import unittest

from candapter import parse_candapter_frame
from protocol import (
    CANFrame,
    decode_frame,
    decoded_time_series,
    fault_status_rows,
    format_fault_raw_reading,
    format_fault_reading,
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
        for payload in (
            b"\x00", b"\x01", b"\x00\x00", b"\x01\x02",
            bytes.fromhex("01 01 A6 04 00 00 88 40"),
            bytes(8),
        ):
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
                if len(payload) < 8:
                    self.assertEqual(
                        decoded["fields"]["Reading at fault"], "Not reported"
                    )
                    self.assertEqual(
                        decoded["fields"]["Raw reading at fault"], "Not reported"
                    )
                row = next(
                    row for row in fault_status_rows({0x505: frame})
                    if row["Fault"] == "Powertrain combined fault"
                )
                self.assertEqual(row["Status"], "FAULT" if active else "OK")

    def test_invalid_fault_status_is_unknown(self) -> None:
        for length in (0, 3, 4, 5, 6, 7, 9):
            payload = b"\x01" * length
            with self.subTest(payload=payload):
                frame = CANFrame(0x505, payload, time.time())
                self.assertTrue(decode_frame(frame)["error"])
                row = next(
                    row for row in fault_status_rows({0x505: frame})
                    if row["Fault"] == "Powertrain combined fault"
                )
                self.assertEqual(row["Status"], "UNKNOWN")

    def test_fault_snapshot_decodes_wire_layout(self) -> None:
        # Independent literal wire bytes catch both endianness conventions.
        frame = CANFrame(
            0x505, bytes.fromhex("01 01 A6 04 00 00 88 40"), time.time()
        )
        decoded = decode_frame(frame)
        fields = decoded["fields"]
        self.assertEqual(decoded["error"], "")
        self.assertEqual(fields["Violated parameter"], "Cell overvoltage")
        self.assertEqual(fields["Reading at fault"], 4.25)
        self.assertEqual(fields["Raw reading at fault"], 42500)
        self.assertEqual(format_fault_reading(fields), "4.2500 V")
        self.assertEqual(format_fault_raw_reading(fields), "42500 (0xA604)")
        self.assertEqual(fields["Fault limit"], "Maximum 4.18 V")

    def test_fault_snapshot_parameters_and_units(self) -> None:
        cases = (
            (2, 25000, 2.5, "2.5000 V", "Minimum 2.52 V"),
            (3, 0x8190, 40.0, "40.0 A", "Maximum 35 A (magnitude)"),
            (3, 0x7E70, 40.0, "40.0 A", "Maximum 35 A (magnitude)"),
            (4, 57, 57.0, "57 \u00b0C", "Maximum 56 \u00b0C"),
            (4, 0xFFFF, 65535.0, "65535 \u00b0C", "Maximum 56 \u00b0C"),
        )
        for parameter, raw, value, rendered, limit in cases:
            with self.subTest(parameter=parameter, raw=raw):
                payload = bytes([1, parameter]) + raw.to_bytes(2, "big")
                payload += struct.pack("<f", value)
                decoded = decode_frame(CANFrame(0x505, payload, time.time()))
                self.assertEqual(decoded["error"], "")
                self.assertEqual(decoded["fields"]["Reading at fault"], value)
                self.assertEqual(decoded["fields"]["Raw reading at fault"], raw)
                self.assertEqual(format_fault_reading(decoded["fields"]), rendered)
                self.assertEqual(decoded["fields"]["Fault limit"], limit)

    def test_no_parameter_means_no_snapshot(self) -> None:
        for status in (0, 1):
            with self.subTest(status=status):
                # Even leftover nonzero data must not invent an estop reading.
                payload = bytes([status, 0]) + bytes.fromhex("FF FF 00 00 88 40")
                decoded = decode_frame(CANFrame(0x505, payload, time.time()))
                self.assertEqual(decoded["error"], "")
                self.assertEqual(decoded["fields"]["Reading at fault"], "Not reported")
                self.assertEqual(
                    decoded["fields"]["Raw reading at fault"], "Not reported"
                )

    def test_unknown_fault_parameter_keeps_raw_and_unknown_units(self) -> None:
        payload = bytes.fromhex("01 FE 12 34 00 00 88 40")
        decoded = decode_frame(CANFrame(0x505, payload, time.time()))
        self.assertEqual(decoded["error"], "")
        self.assertEqual(decoded["fields"]["Violated parameter"], "Unknown (0xFE)")
        self.assertEqual(decoded["fields"]["Raw reading at fault"], 0x1234)
        self.assertEqual(format_fault_reading(decoded["fields"]), "4.25 Unknown unit")
        self.assertEqual(decoded["fields"]["Fault limit"], "Unknown")

    def test_nonfinite_snapshot_retains_fault_and_raw(self) -> None:
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                payload = bytes.fromhex("01 03 FF FF") + struct.pack("<f", value)
                frame = CANFrame(0x505, payload, time.time())
                decoded = decode_frame(frame)
                self.assertEqual(decoded["error"], "")
                self.assertTrue(decoded["fields"]["Fault active"])
                self.assertEqual(
                    format_fault_reading(decoded["fields"]), "Invalid (non-finite)"
                )
                self.assertEqual(decoded["fields"]["Raw reading at fault"], 0xFFFF)
                row = fault_status_rows({0x505: frame})[-1]
                self.assertEqual(row["Status"], "FAULT")

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
