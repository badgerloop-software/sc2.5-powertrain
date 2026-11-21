"""Host regression tests for the production firmware's fault CAN snapshots.

Run from the repository root:
    python -m unittest discover -s test -p test_firmware_faults.py -v

Requires a C++17 compiler; set CXX to its executable path if it is not on PATH.
No PlatformIO install, MCU, CAN adapter, or extra Python packages are needed.
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "test" / "host"


class FirmwareFaultSnapshotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise RuntimeError("Install g++/clang++ or set CXX to run firmware host tests")
        cls.build = tempfile.TemporaryDirectory(prefix="powertrain-fault-tests-")
        cls.addClassCleanup(cls.build.cleanup)
        cls.executable = Path(cls.build.name) / (
            "fault_snapshot_test.exe" if os.name == "nt" else "fault_snapshot_test"
        )
        command = [
            compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(HOST / "stubs"), "-I", str(ROOT / "include"),
            str(ROOT / "src" / "canPowertrain.cpp"),
            str(HOST / "fault_snapshot_test.cpp"), "-o", str(cls.executable),
        ]
        compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
        if compiled.returncode:
            raise RuntimeError(f"Firmware host build failed:\n{compiled.stdout}{compiled.stderr}")

    def run_case(self, name: str) -> None:
        result = subprocess.run(
            [str(self.executable), name], capture_output=True, text=True, timeout=10
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_no_fault_and_estop(self) -> None:
        self.run_case("no_fault_and_estop")

    def test_startup_grace(self) -> None:
        self.run_case("startup_grace")

    def test_inclusive_limits(self) -> None:
        self.run_case("inclusive_limits")

    def test_voltage_fields_and_raw_byte_boundaries(self) -> None:
        self.run_case("voltage_fields")

    def test_current_encoding_on_both_sides_of_zero(self) -> None:
        self.run_case("current_encoding")

    def test_temperature_fields_and_raw_byte_boundaries(self) -> None:
        self.run_case("temperature_fields")

    def test_transient_recovers_before_next_send(self) -> None:
        self.run_case("transient_before_send")

    def test_first_snapshot_is_immutable(self) -> None:
        self.run_case("immutable_snapshot")

    def test_restored_latch_does_not_invent_a_snapshot(self) -> None:
        self.run_case("restored_latch_has_no_snapshot")

    def test_snapshot_is_not_persisted_across_reboot(self) -> None:
        self.run_case("snapshot_not_persisted")

    def test_clear_command_arms_clear_on_restart(self) -> None:
        self.run_case("clear_arms_restart")

    def test_active_fault_cancels_clear_without_changing_snapshot(self) -> None:
        self.run_case("active_fault_cancels_clear")

    def test_malformed_frames_are_ignored(self) -> None:
        self.run_case("malformed_frames_ignored")


if __name__ == "__main__":
    unittest.main()
