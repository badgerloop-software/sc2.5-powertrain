"""SC2 powertrain CAN protocol definitions and decoders."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import struct
from typing import Any


BPS_STATUS_CAN_ID = 0x101
BPS_TEMPERATURE_CAN_ID = 0x108
BPS_ELECTRICAL_CAN_ID = 0x109
POWERTRAIN_ESTOP_FAULT_CAN_ID = 0x505
BPS_UNDERVOLTAGE_FAULT_CAN_ID = 0x506
BPS_OVERVOLTAGE_FAULT_CAN_ID = 0x507
BPS_OVERCURRENT_FAULT_CAN_ID = 0x508
BPS_OVERTEMPERATURE_FAULT_CAN_ID = 0x509
FAULT_CLEAR_CAN_ID = 0x7EB

FAULT_CLEAR_PAYLOAD = bytes.fromhex("03 7F 20 22 00 00 00 00")

FAULT_STATUS_MESSAGES = {
    POWERTRAIN_ESTOP_FAULT_CAN_ID: "E-stop",
    BPS_UNDERVOLTAGE_FAULT_CAN_ID: "Cell undervoltage",
    BPS_OVERVOLTAGE_FAULT_CAN_ID: "Cell overvoltage",
    BPS_OVERCURRENT_FAULT_CAN_ID: "Pack overcurrent",
    BPS_OVERTEMPERATURE_FAULT_CAN_ID: "Overtemperature",
}

CELL_VOLTAGE_MIN_V = 2.52
CELL_VOLTAGE_MAX_V = 4.18
PACK_CURRENT_MAX_A = 35.0
TEMPERATURE_MIN_C = 4.0
TEMPERATURE_MAX_C = 56.0

CELL_VOLTAGE_SCALE_V = 0.0001
PACK_CURRENT_ZERO_RAW = 0x8000
PACK_CURRENT_SCALE_A = 0.1
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
        elif frame.arbitration_id in FAULT_STATUS_MESSAGES:
            if frame.dlc != 1:
                raise ValueError("expected DLC 1")
            active = bool(frame.data[0] & 0x01)
            result["name"] = FAULT_STATUS_MESSAGES[frame.arbitration_id]
            result["fields"] = {
                "Fault active": active,
                "Status": "FAULT" if active else "Healthy",
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


def fault_status_rows(latest: dict[int, CANFrame]) -> list[dict[str, Any]]:
    """Return every known fault condition with its current status."""
    rows: list[dict[str, Any]] = []

    def status_from_message(can_id: int) -> str | None:
        frame = latest.get(can_id)
        if frame is None or frame.dlc != 1:
            return None
        return "FAULT" if frame.data[0] & 0x01 else "OK"

    def add(
        fault: str,
        source: str,
        value: str,
        is_fault: bool | str | None,
        limit: str,
    ) -> None:
        rows.append(
            {
                "Fault": fault,
                "Source": source,
                "Value": value,
                "Limit": limit,
                "Status": (
                    "UNKNOWN"
                    if is_fault is None
                    else (
                        is_fault
                        if isinstance(is_fault, str)
                        else ("FAULT" if is_fault else "OK")
                    )
                ),
            }
        )

    temp = latest.get(BPS_TEMPERATURE_CAN_ID)
    if temp is None or temp.dlc != 4:
        for label in ("Low temperature", "High temperature"):
            add(
                label,
                "0x108",
                "—",
                None,
                f"{TEMPERATURE_MIN_C:g}–{TEMPERATURE_MAX_C:g} °C",
            )
    else:
        low_temp = float(_u16_be(temp.data, 0))
        high_temp = float(_u16_be(temp.data, 2))
        add(
            "Low temperature",
            "0x108",
            f"{low_temp:.0f} °C",
            not TEMPERATURE_MIN_C <= low_temp <= TEMPERATURE_MAX_C,
            f"{TEMPERATURE_MIN_C:g}–{TEMPERATURE_MAX_C:g} °C",
        )
        add(
            "High temperature",
            "0x108",
            f"{high_temp:.0f} °C",
            status_from_message(BPS_OVERTEMPERATURE_FAULT_CAN_ID)
            or not TEMPERATURE_MIN_C <= high_temp <= TEMPERATURE_MAX_C,
            f"{TEMPERATURE_MIN_C:g}–{TEMPERATURE_MAX_C:g} °C",
        )

    electrical = latest.get(BPS_ELECTRICAL_CAN_ID)
    if electrical is None or electrical.dlc != 6:
        add(
            "Cell overvoltage",
            "0x109",
            "—",
            status_from_message(BPS_OVERVOLTAGE_FAULT_CAN_ID),
            f"≤ {CELL_VOLTAGE_MAX_V:.2f} V",
        )
        add(
            "Cell undervoltage",
            "0x109",
            "—",
            status_from_message(BPS_UNDERVOLTAGE_FAULT_CAN_ID),
            f"≥ {CELL_VOLTAGE_MIN_V:.2f} V",
        )
        add(
            "Pack overcurrent",
            "0x109",
            "—",
            status_from_message(BPS_OVERCURRENT_FAULT_CAN_ID),
            f"≤ {PACK_CURRENT_MAX_A:g} A",
        )
    else:
        high_v = _u16_be(electrical.data, 0) * CELL_VOLTAGE_SCALE_V
        low_v = _u16_be(electrical.data, 2) * CELL_VOLTAGE_SCALE_V
        raw_current = _u16_be(electrical.data, 4)
        current = (
            abs(raw_current - PACK_CURRENT_ZERO_RAW)
            * PACK_CURRENT_SCALE_A
        )
        add(
            "Cell overvoltage",
            "0x109",
            f"{high_v:.4f} V",
            status_from_message(BPS_OVERVOLTAGE_FAULT_CAN_ID)
            or high_v > CELL_VOLTAGE_MAX_V
            or low_v > CELL_VOLTAGE_MAX_V,
            f"≤ {CELL_VOLTAGE_MAX_V:.2f} V",
        )
        add(
            "Cell undervoltage",
            "0x109",
            f"{low_v:.4f} V",
            status_from_message(BPS_UNDERVOLTAGE_FAULT_CAN_ID)
            or low_v < CELL_VOLTAGE_MIN_V
            or high_v < CELL_VOLTAGE_MIN_V,
            f"≥ {CELL_VOLTAGE_MIN_V:.2f} V",
        )
        add(
            "Pack overcurrent",
            "0x109",
            f"{current:.1f} A",
            status_from_message(BPS_OVERCURRENT_FAULT_CAN_ID)
            or current > PACK_CURRENT_MAX_A,
            f"≤ {PACK_CURRENT_MAX_A:g} A",
        )

    add(
        "E-stop",
        "0x505",
        "—",
        status_from_message(POWERTRAIN_ESTOP_FAULT_CAN_ID),
        "inactive",
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
