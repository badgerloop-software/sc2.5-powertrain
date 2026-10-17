"""Streamlit CANdapter diagnostic application for SC2 powertrain."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time

import streamlit as st
from streamlit_autorefresh import st_autorefresh

from candapter import CANdapter, CANdapterError
from capture_store import list_captures, load_capture, save_capture
from protocol import (
    BPS_ELECTRICAL_CAN_ID,
    BPS_STATUS_CAN_ID,
    BPS_TEMPERATURE_CAN_ID,
    CANFrame,
    FAULT_CLEAR_CAN_ID,
    FAULT_CLEAR_PAYLOAD,
    POWERTRAIN_FAULT_CAN_ID,
    TIME_SERIES_SIGNALS,
    decode_frame,
    decoded_time_series,
    fault_status_rows,
    pack_soc_trend,
    parse_hex_data,
)


st.set_page_config(
    page_title="SC2 CAN Diagnostics",
    page_icon="🔋",
    layout="wide",
)


def get_adapter() -> CANdapter | None:
    return st.session_state.get("candapter")


def disconnect() -> None:
    adapter = get_adapter()
    if adapter is not None:
        adapter.close()
    st.session_state.candapter = None


@st.cache_data(show_spinner=False)
def cached_load_capture(
    path_text: str,
    modified_time_ns: int,
) -> list[CANFrame]:
    """Load a capture until its file modification time changes."""
    del modified_time_ns
    return load_capture(Path(path_text))


def format_decoded_fields(decoded: dict) -> str:
    fields = decoded["fields"]
    formatted = []
    for name, value in fields.items():
        if "voltage" in name.lower() and isinstance(value, float):
            rendered = f"{value:.4f} V"
        elif "current" in name.lower() and isinstance(value, float):
            rendered = f"{value:.1f} A"
        elif "charge" in name.lower() and isinstance(value, float):
            rendered = f"{value:.1f}%"
        elif "health" in name.lower() and isinstance(value, float):
            rendered = f"{value:.0f}%"
        else:
            rendered = str(value)
        formatted.append(f"{name}: {rendered}")
    return ", ".join(formatted)


if "candapter" not in st.session_state:
    st.session_state.candapter = None
if "event_log" not in st.session_state:
    st.session_state.event_log = []
if "fault_capture_active" not in st.session_state:
    st.session_state.fault_capture_active = False


st.title("SC2 CAN Diagnostics")
st.caption(
    "Ewert Energy Systems / Orion BMS CANdapter monitor, decoder, "
    "and powertrain fault-reset utility"
)
st.markdown(
    """
    <style>
    .block-container {padding-top: 2rem; padding-bottom: 3rem;}
    [data-testid="stMetric"] {
        background: rgba(127, 127, 127, 0.07);
        border: 1px solid rgba(127, 127, 127, 0.18);
        border-radius: 0.65rem;
        padding: 0.75rem 1rem;
    }
    [data-testid="stDataFrame"] {
        border: 1px solid rgba(127, 127, 127, 0.18);
        border-radius: 0.5rem;
    }
    .battery-state {
        display: flex;
        align-items: center;
        gap: 1.2rem;
        padding: 1rem 1.25rem;
        border: 1px solid rgba(127, 127, 127, 0.18);
        border-radius: 0.75rem;
        margin-bottom: 1rem;
    }
    .battery-icon {
        position: relative;
        width: 150px;
        height: 64px;
        border: 5px solid currentColor;
        border-radius: 9px;
        overflow: visible;
    }
    .battery-icon::after {
        content: "";
        position: absolute;
        right: -14px;
        top: 17px;
        width: 9px;
        height: 24px;
        border-radius: 0 4px 4px 0;
        background: currentColor;
    }
    .battery-fill {
        height: 100%;
        border-radius: 3px;
        background: #22c55e;
    }
    .battery-percent {
        font-size: 2.6rem;
        font-weight: 750;
        line-height: 1;
    }
    .charge-direction {
        font-size: 1.35rem;
        font-weight: 700;
        margin-top: 0.45rem;
    }
    .charging {color: #22c55e;}
    .discharging {color: #f59e0b;}
    .holding {color: #3b82f6;}
    .unknown {color: #9ca3af;}
    </style>
    """,
    unsafe_allow_html=True,
)

with st.sidebar:
    st.header("CANdapter connection")
    adapter = get_adapter()
    available_ports = CANdapter.available_ports()
    default_port = available_ports[0] if available_ports else "COM3"
    port = st.selectbox(
        "Serial port",
        available_ports or [default_port],
        disabled=adapter is not None,
    )
    serial_baudrate = st.selectbox(
        "Serial baud rate",
        [9600, 115200],
        index=0,
        disabled=adapter is not None,
        help="The CANdapter manual and reference implementation use 9600.",
    )
    can_bitrate = st.selectbox(
        "CAN bitrate",
        [125_000, 250_000, 500_000, 1_000_000],
        index=1,
        format_func=lambda value: f"{value // 1000} kbit/s",
        disabled=adapter is not None,
    )

    if adapter is None:
        if st.button("Connect", type="primary", width="stretch"):
            try:
                new_adapter = CANdapter(
                    port=port,
                    serial_baudrate=serial_baudrate,
                    can_bitrate=can_bitrate,
                )
                new_adapter.connect()
                st.session_state.candapter = new_adapter
                st.session_state.event_log.append(
                    f"Connected to {port} at {can_bitrate} bit/s"
                )
                st.rerun()
            except Exception as exc:
                st.error(f"Connection failed: {exc}")
    else:
        st.success(
            f"Connected: {adapter.port} @ "
            f"{adapter.can_bitrate // 1000} kbit/s"
        )
        if st.button("Disconnect", width="stretch"):
            disconnect()
            st.rerun()

    auto_refresh = st.checkbox("Auto-refresh", value=True)
    if auto_refresh:
        st_autorefresh(interval=500, key="candapter_refresh")

adapter = get_adapter()
frames = adapter.frames_snapshot() if adapter is not None else []
latest: dict[int, CANFrame] = {}
counts: dict[int, int] = {}
for frame in frames:
    latest[frame.arbitration_id] = frame
    counts[frame.arbitration_id] = counts.get(frame.arbitration_id, 0) + 1

soc = None
if BPS_STATUS_CAN_ID in latest:
    soc_message = decode_frame(latest[BPS_STATUS_CAN_ID])
    if not soc_message["error"]:
        soc = soc_message["fields"].get("State of charge")
soc_trend = pack_soc_trend(frames)

fault_rows = fault_status_rows(latest)
fault_count = sum(row["Status"] == "FAULT" for row in fault_rows)
unknown_count = sum(row["Status"] == "UNKNOWN" for row in fault_rows)
fault_active = fault_count > 0
if fault_active and not st.session_state.fault_capture_active and frames:
    try:
        fault_capture_path = save_capture(
            frames,
            filename_prefix="fault_capture",
        )
        st.session_state.event_log.append(
            f"Automatically saved {len(frames)} frames after fault "
            f"detection to {fault_capture_path.name}"
        )
        st.warning(
            "Fault detected: automatically saved "
            f"{len(frames):,} recent frames as "
            f"{fault_capture_path.name}."
        )
    except (OSError, ValueError) as exc:
        st.error(f"Automatic fault capture failed: {exc}")
st.session_state.fault_capture_active = fault_active

metric_1, metric_2, metric_3, metric_4 = st.columns(4)
metric_1.metric("Connection", "Online" if adapter else "Offline")
metric_2.metric("Frames captured", len(frames))
metric_3.metric("Active known faults", fault_count)
metric_4.metric("Unknown statuses", unknown_count)

status_tab, series_tab, messages_tab, transmit_tab, decoder_tab = st.tabs(
    [
        "Overview",
        "Time series",
        "Live messages",
        "Send frame",
        "Decode frame",
    ]
)

with status_tab:
    temperature = (
        decode_frame(latest[BPS_TEMPERATURE_CAN_ID])
        if BPS_TEMPERATURE_CAN_ID in latest
        else None
    )
    electrical = (
        decode_frame(latest[BPS_ELECTRICAL_CAN_ID])
        if BPS_ELECTRICAL_CAN_ID in latest
        else None
    )
    powertrain = (
        decode_frame(latest[POWERTRAIN_FAULT_CAN_ID])
        if POWERTRAIN_FAULT_CAN_ID in latest
        else None
    )

    display_soc = max(0.0, min(100.0, soc)) if soc is not None else 0.0
    soc_text = f"{soc:.1f}%" if soc is not None else "—%"
    direction = soc_trend["state"]
    direction_class = direction.lower()
    direction_label = {
        "CHARGING": "↑ CHARGING",
        "DISCHARGING": "↓ DISCHARGING",
        "HOLDING": "— HOLDING",
        "UNKNOWN": "… WAITING FOR TREND",
    }[direction]
    delta = soc_trend["delta_percent"]
    trend_detail = (
        f"Recent SoC change (60 s window): {delta:+.1f}%"
        if delta is not None
        else "Need at least two SoC samples"
    )
    fill_color = (
        "#ef4444"
        if display_soc < 20
        else ("#f59e0b" if display_soc < 40 else "#22c55e")
    )
    st.markdown(
        f"""
        <div class="battery-state">
            <div class="battery-icon">
                <div class="battery-fill"
                     style="width:{display_soc:.1f}%;
                            background:{fill_color};"></div>
            </div>
            <div>
                <div class="battery-percent">{soc_text}</div>
                <div>State of charge · 0x101 byte 4</div>
                <div class="charge-direction {direction_class}">
                    {direction_label}
                </div>
                <div>{trend_detail}</div>
            </div>
        </div>
        """,
        unsafe_allow_html=True,
    )

    st.subheader("Live system status")
    temperature_col, electrical_col, powertrain_col = st.columns(3)

    with temperature_col:
        with st.container(border=True):
            st.markdown("#### Temperature · `0x108`")
            if temperature and not temperature["error"]:
                fields = temperature["fields"]
                low_col, high_col = st.columns(2)
                low_col.metric(
                    "Lowest",
                    f"{fields['Lowest temperature']} °C",
                )
                high_col.metric(
                    "Highest",
                    f"{fields['Highest temperature']} °C",
                )
            else:
                st.info("Waiting for temperature data")

    with electrical_col:
        with st.container(border=True):
            st.markdown("#### Battery · `0x109`")
            if electrical and not electrical["error"]:
                fields = electrical["fields"]
                voltage_col_1, voltage_col_2 = st.columns(2)
                voltage_col_1.metric(
                    "Highest cell",
                    f"{fields['Highest cell voltage']:.4f} V",
                )
                voltage_col_2.metric(
                    "Lowest cell",
                    f"{fields['Lowest cell voltage']:.4f} V",
                )
                st.metric(
                    "Absolute pack current",
                    f"{fields['Pack current']:.1f} A",
                )
            else:
                st.info("Waiting for electrical data")

    with powertrain_col:
        with st.container(border=True):
            st.markdown("#### Powertrain · `0x505`")
            if powertrain and not powertrain["error"]:
                is_fault = powertrain["fields"]["Fault active"]
                if is_fault:
                    st.error("FAULT ACTIVE")
                else:
                    st.success("HEALTHY")
                st.metric("Fault bit", "1" if is_fault else "0")
                if is_fault:
                    st.write(
                        "Violated parameter: "
                        f"{powertrain['fields']['Violated parameter']}"
                    )
                st.caption(
                    "Combined BPS latch or active estop status"
                )
            else:
                st.info("Waiting for powertrain status")

    st.divider()
    details_col, reset_col = st.columns([2, 1], gap="large")
    with details_col:
        st.subheader("Fault details")
        st.dataframe(
            fault_rows,
            width="stretch",
            hide_index=True,
            column_order=["Status", "Fault", "Value", "Limit", "Source"],
        )
        st.caption(
            "Temperature limits mirror the current firmware configuration. "
            "The firmware applies a two-second startup grace period."
        )

    with reset_col:
        with st.container(border=True):
            st.subheader("Clear BPS latch")
            st.write(
                "Arms the stored BPS fault to clear on the next "
                "powertrain reset."
            )
            st.code(
                "0x7EB · 03 7F 20 22 00 00 00 00",
                language="text",
            )
            confirm_reset = st.checkbox(
                "Vehicle is in a safe state",
                key="confirm_reset",
            )
            if st.button(
                "Send clear command",
                type="primary",
                disabled=adapter is None or not confirm_reset,
                width="stretch",
            ):
                try:
                    assert adapter is not None
                    adapter.send_frame(
                        FAULT_CLEAR_CAN_ID,
                        FAULT_CLEAR_PAYLOAD,
                    )
                    st.session_state.event_log.append(
                        "Sent BPS fault-clear command"
                    )
                    st.success("Command acknowledged.")
                except (CANdapterError, OSError, ValueError) as exc:
                    st.error(f"Command failed: {exc}")

            if st.session_state.event_log:
                with st.expander("Recent actions"):
                    for event in reversed(
                        st.session_state.event_log[-8:]
                    ):
                        st.text(event)

with series_tab:
    st.subheader("Saved capture comparison")
    save_col, buffer_col = st.columns([1, 2])
    with save_col:
        if st.button(
            "Save current capture",
            type="primary",
            disabled=not frames,
            width="stretch",
        ):
            try:
                saved_path = save_capture(frames)
                st.session_state.event_log.append(
                    f"Saved {len(frames)} frames to {saved_path.name}"
                )
                st.success(
                    f"Saved {len(frames):,} frames as {saved_path.name}"
                )
            except (OSError, ValueError) as exc:
                st.error(f"Capture save failed: {exc}")
    with buffer_col:
        st.info(
            f"The live buffer currently contains {len(frames):,} of "
            "5,000 possible frames."
        )

    saved_capture_paths = list_captures()
    capture_options: list[Path | None] = [None, *saved_capture_paths]
    selected_capture = st.selectbox(
        "Saved capture",
        capture_options,
        index=1 if saved_capture_paths else 0,
        format_func=lambda path: (
            "No saved capture (live data only)"
            if path is None
            else path.stem
        ),
    )
    signal_key = st.selectbox(
        "Signal",
        list(TIME_SERIES_SIGNALS),
        format_func=lambda key: TIME_SERIES_SIGNALS[key]["label"],
    )

    saved_frames: list[CANFrame] = []
    if selected_capture is not None:
        try:
            saved_frames = cached_load_capture(
                str(selected_capture),
                selected_capture.stat().st_mtime_ns,
            )
        except (OSError, ValueError) as exc:
            st.error(f"Capture load failed: {exc}")

    saved_points = decoded_time_series(saved_frames, signal_key)
    live_points = decoded_time_series(frames, signal_key)
    combined_points = sorted(
        [*saved_points, *live_points],
        key=lambda point: point["timestamp"],
    )

    saved_metric, live_metric, total_metric = st.columns(3)
    saved_metric.metric(
        "Saved signal points",
        f"{len(saved_points):,}",
    )
    live_metric.metric(
        "Live signal points",
        f"{len(live_points):,}",
    )
    total_metric.metric(
        "Combined signal points",
        f"{len(combined_points):,}",
    )

    if saved_points and live_points:
        gap_seconds = (
            live_points[0]["timestamp"] - saved_points[-1]["timestamp"]
        )
        if gap_seconds > 0:
            st.warning(
                "No CAN data was recorded during the "
                f"{gap_seconds / 60:.1f}-minute gap. The straight line "
                "across that interval is visual interpolation, not a "
                "measurement."
            )

    signal = TIME_SERIES_SIGNALS[signal_key]
    value_column = f"{signal['label']} ({signal['unit']})"
    chart_rows = [
        {
            "Time": datetime.fromtimestamp(point["timestamp"]),
            value_column: point["value"],
        }
        for point in combined_points
    ]
    if chart_rows:
        st.line_chart(
            chart_rows,
            x="Time",
            y=value_column,
            height=450,
            use_container_width=True,
        )
        st.caption(
            f"Charted {len(combined_points):,} matching telemetry points "
            f"from {len(saved_frames) + len(frames):,} raw CAN frames."
        )
    else:
        st.info("No matching signal data is available to graph yet.")

with messages_tab:
    controls_left, controls_right = st.columns([1, 4])
    with controls_left:
        if st.button(
            "Clear capture",
            disabled=adapter is None,
            width="stretch",
        ):
            assert adapter is not None
            adapter.clear_frames()
            st.rerun()
    with controls_right:
        id_filter = st.text_input(
            "Optional CAN ID filter",
            placeholder="Example: 109",
        ).strip()

    latest_rows = []
    for can_id, frame in sorted(latest.items()):
        decoded = decode_frame(frame)
        latest_rows.append(
            {
                "ID": decoded["id"],
                "DLC": decoded["dlc"],
                "Data": decoded["data"],
                "Name": decoded["name"],
                "Decoded fields": format_decoded_fields(decoded),
                "Count": counts[can_id],
                "Last seen": decoded["timestamp"],
                "Error": decoded["error"],
            }
        )

    if id_filter:
        normalized_filter = id_filter.lower().removeprefix("0x")
        latest_rows = [
            row
            for row in latest_rows
            if row["ID"].lower().removeprefix("0x") == normalized_filter
        ]

    st.subheader("Latest frame by CAN ID")
    st.dataframe(latest_rows, width="stretch", hide_index=True)

    raw_rows = []
    for frame in reversed(frames[-250:]):
        decoded = decode_frame(frame)
        raw_rows.append(
            {
                "Time": decoded["timestamp"],
                "ID": decoded["id"],
                "DLC": decoded["dlc"],
                "Data": decoded["data"],
                "Name": decoded["name"],
            }
        )
    st.subheader("Recent raw frames")
    st.dataframe(raw_rows, width="stretch", hide_index=True)

    if adapter is not None and adapter.errors_snapshot():
        with st.expander("CANdapter parser errors"):
            for error in adapter.errors_snapshot():
                st.text(error)

with transmit_tab:
    st.subheader("Transmit custom standard frame")
    tx_id_text = st.text_input("CAN ID (hex)", value="108", key="tx_id")
    tx_data_text = st.text_input(
        "Payload bytes (hex)",
        value="00 18 00 1A",
        key="tx_data",
    )
    st.warning("Custom transmission writes directly to the live CAN bus.")
    custom_confirm = st.checkbox(
        "I confirm this custom frame is safe to transmit",
        key="custom_confirm",
    )
    if st.button(
        "Transmit custom frame",
        disabled=adapter is None or not custom_confirm,
    ):
        try:
            arbitration_id = int(tx_id_text.lower().removeprefix("0x"), 16)
            payload = parse_hex_data(tx_data_text)
            assert adapter is not None
            adapter.send_frame(arbitration_id, payload)
            st.success(
                f"Sent 0x{arbitration_id:03X}: "
                f"{payload.hex(' ').upper()}"
            )
        except (CANdapterError, OSError, ValueError) as exc:
            st.error(f"Transmit failed: {exc}")

with decoder_tab:
    st.subheader("Decode a frame without transmitting")
    decode_id_text = st.text_input(
        "CAN ID (hex)",
        value="109",
        key="decode_id",
    )
    decode_data_text = st.text_input(
        "Payload bytes (hex)",
        value="85 CF 83 3F 7F FF",
        key="decode_data",
    )
    try:
        decode_id = int(decode_id_text.lower().removeprefix("0x"), 16)
        decode_payload = parse_hex_data(decode_data_text)
        offline_frame = CANFrame(
            arbitration_id=decode_id,
            data=decode_payload,
            timestamp=time.time(),
        )
        decoded = decode_frame(offline_frame)
        st.json(decoded)
    except ValueError as exc:
        st.error(str(exc))
