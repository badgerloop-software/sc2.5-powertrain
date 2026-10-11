"""SC2 powertrain CAN protocol definitions and decoders."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import struct
from typing import Any


BPS_STATUS_CAN_ID = 0x101
BPS_TEMPERATURE_CAN_ID = 0x108
BPS_ELECTRICAL_CAN_ID = 0x109
POWERTRAIN_FAULT_CAN_ID = 0x505
POWERTRAIN_FAULT_CAN_DLC = 3
FAULT_CLEAR_CAN_ID = 0x7EB

FAULT_CLEAR_PAYLOAD = bytes.fromhex("03 7F 20 22 00 00 00 00")

FAULT_CODE_NAMES = {
    0x00: "No fault",
    0x01: "E-stop",
    0x02: "Cell undervoltage",
    0x03: "Cell overvoltage",
    0x04: "Pack overcurrent",
    0x05: "Overtemperature",
}

CELL_VOLTAGE_MIN_V = 2.52
CELL_VOLTAGE_MAX_V = 4.18
PACK_CURRENT_MAX_A = 35.0
TEMPERATURE_MIN_C = 4.0
TEMPERATURE_MAX_C = 56.0

CELL_VOLTAGE_SCALE_V = 0.0001
PACK_CURRENT_ZERO_RAW = 0x8000
PACK_CURRENT_SCALE_A = 0.1
FAULT_CURRENT_SCALE_A = PACK_CURRENT_SCALE_A
FAULT_TEMPERATURE_SCALE_C = 1.0
STATE_OF_CHARGE_SCALE_PERCENT = 0.5
SOC_TREND_WINDOW_SECONDS = 60.0

TIME_SERIES_SIGNALS = {
    "pack_soc": {
        "label": "Pack state of charge",
        "unit": "%",
        "can_id": BPS_STATUS_CAN_ID,
        "field": "State of charge",
    },
    "pack_current": {
        "label": "Absolute pack current",
        "unit": "A",
        "can_id": BPS_ELECTRICAL_CAN_ID,
        "field": "Pack current",
    },
    "highest_cell_voltage": {
        "label": "Highest cell voltage",
        "unit": "V",
        "can_id": BPS_ELECTRICAL_CAN_ID,
        "field": "Highest cell voltage",
    },
    "lowest_cell_voltage": {
        "label": "Lowest cell voltage",
        "unit": "V",
        "can_id": BPS_ELECTRICAL_CAN_ID,
        "field": "Lowest cell voltage",
    },
    "lowest_temperature": {
        "label": "Lowest pack temperature",
        "unit": "°C",
        "can_id": BPS_TEMPERATURE_CAN_ID,
        "field": "Lowest temperature",
    },
    "highest_temperature": {
        "label": "Highest pack temperature",
        "unit": "°C",
        "can_id": BPS_TEMPERATURE_CAN_ID,
        "field": "Highest temperature",
    },
    "12v_current": {
        "label": "12 V current",
        "unit": "A",
        "can_id": 0x500,
        "field": "Value",
    },
    "12v_voltage": {
        "label": "12 V voltage",
        "unit": "V",
        "can_id": 0x501,
        "field": "Value",
    },
    "supplemental_current": {
        "label": "Supplemental current",
        "unit": "A",
        "can_id": 0x502,
        "field": "Value",
    },
    "battery_current": {
        "label": "Battery current",
        "unit": "A",
        "can_id": 0x503,
        "field": "Value",
    },
    "supplemental_voltage": {
        "label": "Supplemental voltage",
        "unit": "V",
        "can_id": 0x504,
        "field": "Value",
    },
}


@dataclass(frozen=True)
class CANFrame:
    arbitration_id: int
    data: bytes
    timestamp: float
    is_extended: bool = False

    @property
    def dlc(self) -> int:
        return len(self.data)

    @property
    def id_hex(self) -> str:
        width = 8 if self.is_extended else 3
        return f"{self.arbitration_id:0{width}X}"

    @property
    def data_hex(self) -> str:
        return self.data.hex(" ").upper()


def _u16_be(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "big", signed=False)


def decode_frame(frame: CANFrame) -> dict[str, Any]:
    """Decode a known frame, retaining raw data for unknown IDs."""
    result: dict[str, Any] = {
        "timestamp": datetime.fromtimestamp(frame.timestamp).strftime(
            "%H:%M:%S.%f"
        )[:-3],
        "id": f"0x{frame.id_hex}",
        "dlc": frame.dlc,
        "data": frame.data_hex,
        "name": "Unknown",
        "fields": {},
        "error": "",
    }

    try:
        if frame.arbitration_id == BPS_STATUS_CAN_ID:
            if frame.dlc < 5:
                raise ValueError("expected DLC of at least 5")
            result["name"] = "BPS pack status"
            result["fields"] = {
                "State of charge": (
                    frame.data[4] * STATE_OF_CHARGE_SCALE_PERCENT
                )
            }
        elif frame.arbitration_id == BPS_TEMPERATURE_CAN_ID:
            if frame.dlc != 4:
                raise ValueError("expected DLC 4")
            result["name"] = "BPS temperatures"
            result["fields"] = {
                "Lowest temperature": _u16_be(frame.data, 0),
                "Highest temperature": _u16_be(frame.data, 2),
            }
        elif frame.arbitration_id == BPS_ELECTRICAL_CAN_ID:
            if frame.dlc != 6:
                raise ValueError("expected DLC 6")
            raw_current = _u16_be(frame.data, 4)
            result["name"] = "BPS electrical"
            result["fields"] = {
                "Highest cell voltage": (
                    _u16_be(frame.data, 0) * CELL_VOLTAGE_SCALE_V
                ),
                "Lowest cell voltage": (
                    _u16_be(frame.data, 2) * CELL_VOLTAGE_SCALE_V
                ),
                "Pack current": (
                    abs(raw_current - PACK_CURRENT_ZERO_RAW)
                    * PACK_CURRENT_SCALE_A
                ),
            }
        elif frame.arbitration_id == POWERTRAIN_FAULT_CAN_ID:
            if frame.dlc != POWERTRAIN_FAULT_CAN_DLC:
                raise ValueError(
                    f"expected DLC {POWERTRAIN_FAULT_CAN_DLC}"
                )
            fault_code = frame.data[0]
            raw_value = _u16_be(frame.data, 1)
            result["name"] = "Powertrain fault code"
            if fault_code in {0x02, 0x03}:
                value = raw_value * CELL_VOLTAGE_SCALE_V
                value_text = f"{value:.4f} V"
            elif fault_code == 0x04:
                value = raw_value * FAULT_CURRENT_SCALE_A
                value_text = f"{value:.1f} A"
            elif fault_code == 0x05:
                value = raw_value * FAULT_TEMPERATURE_SCALE_C
                value_text = f"{value:.0f} C"
            else:
                value = raw_value
                value_text = str(value)
            result["fields"] = {
                "Fault code": fault_code,
                "Fault": FAULT_CODE_NAMES.get(
                    fault_code,
                    f"Unknown fault (0x{fault_code:02X})",
                ),
                "Violation value": value_text,
                "Raw violation value": raw_value,
                "Status": "CLEAR" if fault_code == 0 else "FAULT",
            }
        elif frame.arbitration_id == FAULT_CLEAR_CAN_ID:
            result["name"] = "Powertrain fault-clear command"
            result["fields"] = {
                "Exact command": frame.data == FAULT_CLEAR_PAYLOAD
            }
        elif 0x500 <= frame.arbitration_id <= 0x504:
            if frame.dlc != 4:
                raise ValueError("expected DLC 4")
            names = {
                0x500: "12 V current",
                0x501: "12 V voltage",
                0x502: "Supplemental current",
                0x503: "Battery current",
                0x504: "Supplemental voltage",
            }
            result["name"] = names[frame.arbitration_id]
            result["fields"] = {
                "Value": struct.unpack("<f", frame.data)[0]
            }
    except (IndexError, struct.error, ValueError) as exc:
        result["error"] = str(exc)

    return result


def pack_soc_trend(
    frames: list[CANFrame],
    window_seconds: float = SOC_TREND_WINDOW_SECONDS,
) -> dict[str, Any]:
    """Classify the Pack SoC change over the latest rolling time window."""
    if window_seconds <= 0:
        raise ValueError("window_seconds must be positive")

    samples = [
        (
            frame.timestamp,
            frame.data[4] * STATE_OF_CHARGE_SCALE_PERCENT,
        )
        for frame in frames
        if frame.arbitration_id == BPS_STATUS_CAN_ID and frame.dlc >= 5
    ]
    if len(samples) < 2:
        return {"state": "UNKNOWN", "delta_percent": None}

    samples.sort(key=lambda sample: sample[0])
    latest_timestamp = samples[-1][0]
    cutoff = latest_timestamp - window_seconds
    recent_samples = [
        sample for sample in samples if sample[0] >= cutoff
    ]
    if len(recent_samples) < 2:
        return {"state": "UNKNOWN", "delta_percent": None}

    delta = recent_samples[-1][1] - recent_samples[0][1]
    if delta > 0:
        state = "CHARGING"
    elif delta < 0:
        state = "DISCHARGING"
    else:
        state = "HOLDING"

    return {"state": state, "delta_percent": delta}


def decoded_time_series(
    frames: list[CANFrame],
    signal_key: str,
) -> list[dict[str, float]]:
    """Return timestamped numeric values for one known telemetry signal."""
    if signal_key not in TIME_SERIES_SIGNALS:
        raise ValueError(f"Unknown time-series signal: {signal_key}")

    signal = TIME_SERIES_SIGNALS[signal_key]
    points: list[dict[str, float]] = []
    for frame in frames:
        if frame.arbitration_id != signal["can_id"]:
            continue
        decoded = decode_frame(frame)
        if decoded["error"]:
            continue
        value = decoded["fields"].get(signal["field"])
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            points.append(
                {
                    "timestamp": frame.timestamp,
                    "value": float(value),
                }
            )

    return sorted(points, key=lambda point: point["timestamp"])


def fault_status_rows(
    latest: dict[int, CANFrame],
    fault_codes: set[int] | None = None,
    fault_code_clear_seen: bool = False,
) -> list[dict[str, Any]]:
    """Return the known fault-code statuses and current telemetry values."""
    rows: list[dict[str, Any]] = []

    def code_status(code: int) -> str | None:
        if fault_codes is None:
            frame = latest.get(POWERTRAIN_FAULT_CAN_ID)
            if frame is None or frame.dlc != POWERTRAIN_FAULT_CAN_DLC:
                return None
            fault_codes = {frame.data[0]}
            clear_seen = frame.data[0] == 0
        else:
            clear_seen = fault_code_clear_seen
        if code in fault_codes:
            return "FAULT"
        return "OK" if clear_seen else "UNKNOWN"

    def add(
        fault: str,
        source: str,
        value: str,
        status: str | None,
        limit: str,
    ) -> None:
        rows.append(
            {
                "Fault": fault,
                "Source": source,
                "Value": value,
                "Limit": limit,
                "Status": status or "UNKNOWN",
            }
        )

    temp = latest.get(BPS_TEMPERATURE_CAN_ID)
    electrical = latest.get(BPS_ELECTRICAL_CAN_ID)
    if temp is None or temp.dlc != 4:
        temperature_value = "—"
    else:
        temperature_value = f"{_u16_be(temp.data, 2):.0f} °C"
    add(
        "Overtemperature",
        "0x505 code 5",
        temperature_value,
        code_status(0x05),
        f"> {TEMPERATURE_MAX_C:g} °C",
    )

    if electrical is None or electrical.dlc != 6:
        voltage_value = "—"
        current_value = "—"
    else:
        high_v = _u16_be(electrical.data, 0) * CELL_VOLTAGE_SCALE_V
        low_v = _u16_be(electrical.data, 2) * CELL_VOLTAGE_SCALE_V
        raw_current = _u16_be(electrical.data, 4)
        current = (
            abs(raw_current - PACK_CURRENT_ZERO_RAW)
            * PACK_CURRENT_SCALE_A
        )
        voltage_value = f"{low_v:.4f}–{high_v:.4f} V"
        current_value = f"{current:.1f} A"

    add(
        "Cell undervoltage",
        "0x505 code 2",
        voltage_value,
        code_status(0x02),
        f"< {CELL_VOLTAGE_MIN_V:.2f} V",
    )
    add(
        "Cell overvoltage",
        "0x505 code 3",
        voltage_value,
        code_status(0x03),
        f"> {CELL_VOLTAGE_MAX_V:.2f} V",
    )
    add(
        "Pack overcurrent",
        "0x505 code 4",
        current_value,
        code_status(0x04),
        f"> {PACK_CURRENT_MAX_A:g} A",
    )
    add(
        "E-stop",
        "0x505 code 1",
        "—",
        code_status(0x01),
        "active-low input",
    )

    return rows


def parse_hex_data(value: str) -> bytes:
    compact = (
        value.replace("0x", "")
        .replace("0X", "")
        .replace(" ", "")
        .replace(",", "")
        .replace("_", "")
    )
    if len(compact) % 2:
        raise ValueError("Data must contain complete byte pairs")
    data = bytes.fromhex(compact)
    if len(data) > 8:
        raise ValueError("Classic CAN payloads are limited to 8 bytes")
    return data
