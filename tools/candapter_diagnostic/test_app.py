"""Exercise the displayed fault snapshot and delayed automatic capture."""

from pathlib import Path
import time
import unittest
from unittest.mock import patch

from streamlit.testing.v1 import AppTest

from protocol import CANFrame


class FakeAdapter:
    port = "TEST"
    can_bitrate = 250_000

    def __init__(self, frames: list[CANFrame]) -> None:
        self.frames = frames

    def frames_snapshot(self) -> list[CANFrame]:
        return list(self.frames)

    def errors_snapshot(self) -> list[str]:
        return []


class FaultSnapshotAppTests(unittest.TestCase):
    def run_app(self, adapter: FakeAdapter) -> AppTest:
        app = AppTest.from_file(
            str(Path(__file__).with_name("app.py")), default_timeout=15
        )
        app.session_state["candapter"] = adapter
        app.run()
        self.assertEqual(len(app.exception), 0)
        return app

    @patch("candapter.CANdapter.available_ports", return_value=[])
    @patch("capture_store.save_capture", return_value=Path("fault_capture_test.csv"))
    def test_snapshot_remains_visible_after_live_reading_recovers(
        self, save_capture, available_ports
    ) -> None:
        now = time.time()
        adapter = FakeAdapter([
            CANFrame(0x505, bytes.fromhex("01 01 A6 04 00 00 88 40"), now),
            CANFrame(0x109, bytes.fromhex("85 CF 83 3F 7F FF"), now + 1),
        ])
        app = self.run_app(adapter)
        metrics = {metric.label: metric.value for metric in app.metric}
        self.assertEqual(metrics["Highest cell"], "3.4255 V")
        self.assertEqual(metrics["Reading at fault"], "4.2500 V")
        self.assertIn(
            "Raw reading: 42500 (0xA604)",
            [caption.value for caption in app.caption],
        )
        self.assertIn(
            "Configured limit: Maximum 4.18 V",
            [caption.value for caption in app.caption],
        )
        self.assertIn("4.2500 V", app.session_state["event_log"][-1])
        self.assertEqual(save_capture.call_count, 1)

    @patch("candapter.CANdapter.available_ports", return_value=[])
    @patch("capture_store.save_capture", return_value=Path("fault_capture_test.csv"))
    def test_legacy_frame_shows_no_invented_reading(
        self, save_capture, available_ports
    ) -> None:
        app = self.run_app(FakeAdapter([CANFrame(0x505, b"\x01\x01", time.time())]))
        metrics = {metric.label: metric.value for metric in app.metric}
        self.assertEqual(metrics["Reading at fault"], "Not reported")
        self.assertIn(
            "Raw reading: Not reported", [caption.value for caption in app.caption]
        )

    @patch("candapter.CANdapter.available_ports", return_value=[])
    @patch("capture_store.save_capture", return_value=Path("fault_capture_test.csv"))
    def test_delayed_snapshot_saved_once_after_initial_live_fault(
        self, save_capture, available_ports
    ) -> None:
        now = time.time()
        adapter = FakeAdapter([
            CANFrame(0x109, bytes.fromhex("A6 04 83 3F 7F FF"), now),
        ])
        app = self.run_app(adapter)
        self.assertEqual(save_capture.call_count, 1)

        adapter.frames.append(
            CANFrame(0x505, bytes.fromhex("01 01 A6 04 00 00 88 40"), now + 1)
        )
        app.run()
        self.assertEqual(len(app.exception), 0)
        self.assertEqual(save_capture.call_count, 2)
        self.assertIn("raw 42500 (0xA604)", app.session_state["event_log"][-1])

        app.run()
        self.assertEqual(len(app.exception), 0)
        self.assertEqual(save_capture.call_count, 2)


if __name__ == "__main__":
    unittest.main()
