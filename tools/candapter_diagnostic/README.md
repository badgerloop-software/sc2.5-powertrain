# SC2 CANdapter Diagnostic App

Streamlit utility for monitoring and decoding the SC2 powertrain CAN bus
through an Ewert Energy Systems / Orion BMS CANdapter.

## Features

- Connects to the CANdapter virtual COM port at 250 kbit/s CAN
- Displays every received standard or extended CAN frame
- Decodes powertrain IDs `0x101`, `0x108`, `0x109`, and `0x505`
- Decodes float telemetry on IDs `0x500` through `0x504`
- Shows whether the pack SoC is charging, discharging, or holding over a
  rolling 60-second window
- Saves the current 5,000-frame buffer locally and graphs a saved capture
  together with the current live buffer
- Automatically saves the available recent buffer when a known fault first
  becomes active
- Shows each known BPS limit and live fault status
- Sends the exact persistent-fault reset command on `0x7EB`
- Provides guarded custom-frame transmission and an offline decoder

## Install and run

From this directory:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
py -m streamlit run app.py
```

Close the Orion BMS utility and CANdapter utility before connecting; only one
application can own the COM port at a time.

The default settings are:

- Serial baud: 9600
- CAN bitrate: 250 kbit/s
- Standard 11-bit CAN identifiers

The CANdapter must be wired to the same powered and properly terminated CAN
bus as the powertrain board. The CANdapter DB9 uses pin 3 for CAN High and pin
5 for CAN Low according to the manufacturer's documentation.

## Saved capture comparison

Use the **Time series** tab to compare telemetry before and after a drive:

1. Allow the live buffer to collect up to 5,000 frames.
2. Select **Save current capture**. The app writes a timestamped CSV to the
   local `captures/` directory.
3. Disconnect or unplug the CANdapter, drive the car, then reconnect it and
   collect the second live buffer.
4. Select the saved capture and a decoded signal. The graph combines matching
   points from the saved and live buffers in timestamp order.

The CANdapter records nothing while it is unplugged. The graph draws a straight
line between the last saved point and first live point, but that segment is
only visual interpolation across the missing interval. Capture CSV files are
kept locally and excluded from Git.

When any known battery or combined powertrain fault transitions from clear to
active, the app immediately writes the available buffer to a
`fault_capture_*.csv` file. It saves only once while that fault remains active
and rearms after all known faults clear, preventing a new file on every
auto-refresh.

## Fault reset behavior

The guarded reset button sends:

```text
ID:      0x7EB
DLC:     8
Payload: 03 7F 20 22 00 00 00 00
```

This arms the current powertrain firmware to clear its persistent BPS latch
on the next powertrain power/reset cycle. An active BPS condition cancels the
pending clear. Estop is transient and is not stored in EEPROM.

## Protocol assumptions

- `0x101`: Pack SoC in zero-based byte 4 at 0.5%/bit
- `0x108`: two big-endian unsigned temperatures in whole degrees Celsius
- `0x109`: two big-endian cell voltages at 0.1 mV/bit, followed by pack
  current magnitude at 0.1 A/bit around a raw zero point of `0x8000`
- `0x505`: byte 0 bit 0 is the combined fault status (`1` means fault);
  optional byte 1 identifies the violated BPS limit. Both legacy one-byte
  and current two-byte frames are supported; legacy frames show the fault
  reason as "Not reported".

Keep `protocol.py` synchronized with `include/powertrainConfig.h` whenever
the firmware limits or message definitions change.
