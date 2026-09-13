# Octave receiver

## Reusable client and application adapters

The protocol implementation now lives in
[`../esp_timeseries_octave`](../esp_timeseries_octave/README.md). It provides
`esp_ts_*` functions for connection, commands, capture metadata, framed binary
or retryable hex blocks, CRC checking, decoding, waiting, and explicit saving
and clearing. It has no motor, PID, or angular-LUT dependency.

This directory retains the textual motor console, calibration workflow, plotting,
and current/fault interpretation. Generic operations call `esp_ts_*` directly.
`ts_capture` adds application interpretation and optional save/plot/clear around
the generic downloader. `ts_load_timeseries_tools` loads the library
automatically, so `ts_console` usage is unchanged.

The library is an independent MIT-licensed repository, included as a submodule.
Run `git submodule update --init --recursive` after updating this project.
Installation uses `addpath`, not an Octave `pkg install` archive.

### Removed compatibility adapters

The following forwarding-only functions were removed. Update external scripts
using this mapping and call `ts_load_timeseries_tools()` after adding
`tools/octave` to the path (or add the library's `inst/` directory directly).
The console and standalone calibration entry points load the library themselves.

| Removed function | Replacement |
|---|---|
| `ts_command` | `esp_ts_command` |
| `ts_open_serial` | `esp_ts_open` |
| `ts_list_serial_ports` | `esp_ts_ports` |
| `ts_load_instrument_control` | `esp_ts_load_instrument_control` |
| `ts_decode_payload` | `esp_ts_decode_payload` |
| `ts_crc32_ieee` | `esp_ts_crc32` |

## Running the motor console

Requires GNU Octave and the instrument-control package version providing the
serialport API:

    pkg install -forge instrument-control

List ports with:

    pkg load instrument-control
    serialportlist()

On macOS the native ESP32-S3 port normally resembles
/dev/cu.usbmodem...; on Linux it normally resembles /dev/ttyACM....
The interactive console automatically converts macOS `/dev/tty.*` entries
reported by `serialportlist()` to their `/dev/cu.*` call-out counterparts.

Flash and monitor the firmware through the board's USB-UART/programming
connector. Run these commands against the other connector, wired directly to
the ESP32-S3 native USB peripheral. The monitor may remain open because its
UART logs are physically separate from the capture stream.

Examples:

    addpath("tools/octave")
    ts_console()

`ts_console` is the recommended interactive entry point. It keeps one serial
connection open, lists the ports, and provides a complete experiment action.
All interactive menus and confirmations are textual and remain in the Octave
terminal. Graphs are the only windows opened by the console.
On Windows, opening the native ESP32-S3 COM port may briefly restart the board.
The console drains this boot text and retries `STATUS`; it enters the recorder
menu only after receiving a complete `OK command=STATUS` response. A port that
continues to emit `I (...)`, `W (...)`, or `E (...)` logs is rejected as UART.
Before sending `ARM`, that action reads the volatile controller configuration
from the ESP32 and shows `Kp`, `Ki`, `Kd`, and the reference-step period. The
parameter choice uses a text menu in the terminal and prints each value on its
own line. The user can keep the values, replace all four, restore the compiled
defaults, or cancel. Accepted values are copied into the controller only when
that `ARM` starts the experiment; they are never written to NVS and return to
their firmware defaults after reset or power loss. The selected values are also
saved as `capture.control_config` in the MAT file.

The action then sends `ARM`, polls `STATUS` every 2 seconds, automatically starts
fast `DUMP FRAMED` binary transfer on macOS/Linux or the retryable `DUMP
BEGIN`/`DUMP BLOCK`/`DUMP END` sequence on Windows. It validates the framing,
per-block checks where applicable, and the overall CRC, saves a MAT file, and
opens one graph
with four linked panels: reference plus speed, speed error, control action, and
current. Current samples explicitly marked invalid or saturated by the firmware
are replaced for display by the mean of their valid neighbors and highlighted
with a red circle. The saved raw and scaled data are never modified. Individual
protocol operations remain in an advanced menu.

The console also provides an end-to-end angular calibration assistant.
It runs the synchronized 40/55/40 percent open-loop profile at 500 Hz, extracts
complete steady-state revolutions, estimates a 256-bin cyclic correction LUT,
and validates the result on alternating revolutions that were not used to fit
the table. The result window shows the correction and phase error before/after.
The MAT file contains both the original `capture` and calculated `calibration`.

After user confirmation, Octave uploads the signed native-count corrections and
their counts-per-revolution scale as binary, checks the IEEE CRC32, reads the
table back byte-for-byte, and only then enables it. Format 2 accepts
power-of-two scales from 256 through 65536 counts; the default remains 16384
for the MT6701. The firmware stores two versioned NVS slots so an interrupted
write cannot replace the last valid table. New uploads start disabled.
Calibration management in the console can inspect, enable, disable, read back,
or erase the stored LUT.

For an open-loop capture containing the raw `angle_raw` channel (legacy files
named it `angle`), the panels change
to speed, wrapped sensor angle, control duty, and current. The console prints
the predicted/actual acquisition duration separately from the measured USB DUMP
duration, transport mode, and KiB/s. On Windows, the receiver requests at most
512 raw bytes per command; each block arrives as an identified hexadecimal line
with its own CRC and can be retried independently. This is slower than the
continuous binary stream retained on macOS/Linux, but avoids the Windows
driver's ambiguous retained-tail behavior and prevents text responses from
being mistaken for samples. With
the current five-channel diagnostic build, select
250 Hz for a long diagnostic trace. The calibration assistant uses 500 Hz and
the calibration firmware profile uses three eight-second duty stages, followed
by BRAKE before the five-channel 128 KiB buffer becomes full.

Calibration protocol commands are also available directly:

    ts_calibration_status(port)
    installed = ts_calibration_read(port)
    status = ts_calibration_write(port, calibration, true)

The final argument above requests enablement only after CRC and readback
verification. Firmware commands are `CAL START`, `CAL STATUS`, `CAL WRITE`, `CAL READ`,
`CAL ENABLE`, `CAL DISABLE`, and `CAL CLEAR`.
Before transmitting a LUT, `ts_calibration_write` queries the firmware and
prints a preflight summary of correction magnitude and corrected-step limits
using the same acceptance rules as the embedded component.

The plot font size is configurable. Omitting it uses 12 points:

    ts_plot_capture(capture, 16)

The `current` channel is signed and follows the effective bridge direction:
positive drive uses `R_IS`, while negative drive uses negated `L_IS`. Tagged
values in the same `int16` distinguish `R_IS`, `L_IS`, and simultaneous faults,
as well as `BRAKE`, `COAST`, and unavailable measurements without consuming a
sixth channel. The client exposes these masks in `capture.current_status` and
replaces tagged engineering values with `NaN`.

The current plot replaces only fault spikes visually and uses different red
markers for `R_IS`, `L_IS`, and simultaneous faults. The control subplot marks
explicit low-side `BRAKE` samples in magenta and marks torque opposed to the
measured direction in orange. `BRAKE` and `COAST` remain gaps in the current
trace because the high-side sense outputs do not reliably measure armature
current in those modes.

The `bts7960-current-v1` raw codes are:

| Raw `int16` | Meaning |
| ---: | --- |
| 32760 | explicit `BRAKE` |
| 32761 | `COAST` |
| 32762 | `R_IS` fault |
| 32763 | `L_IS` fault |
| 32764 | simultaneous `R_IS` and `L_IS` fault |
| 32765 | measurement unavailable |

Valid current is limited to ±32000 mA, leaving the tagged range unambiguous.

The lower-level functions remain available for automated experiments:

    ts_load_timeseries_tools()
    device = esp_ts_open("/dev/cu.usbmodem1101", 30)
    esp_ts_command(device, "STATUS")
    esp_ts_arm(device, 250)
    esp_ts_wait_full(device, 2, 120)
    capture = ts_capture(device, "capture.mat")
    esp_ts_clear(device)

Keep the same connection throughout the experiment. Opening another connection
can reset the board on some hosts. These low-level calls do not replace the
console's boot synchronization or its explicit motor-stop handling.

The application-specific volatile controller commands are:

    esp_ts_command(device, "CONTROL GET")
    esp_ts_command(device, "CONTROL SET 0.25 3.0 0.0001 2.0")
    esp_ts_command(device, "CONTROL DEFAULTS")

The firmware accepts `0 <= Kp <= 100`, `0 <= Ki <= 1000`,
`0 <= Kd <= 10`, and `0.1 <= reference_step_period_s <= 3600`. These bounds
validate the protocol; they are not a claim that every accepted combination is
safe or stable for the connected motor.

This application deliberately gives the host exclusive ownership of the
ARM/CLEAR sequence; there is no application-level automatic pretrigger mode.
Do not leave another serial monitor connected to the native USB port: two
readers split protocol bytes. Use the WCH/USB-UART port for firmware logs.
Leaving the console sends `CONTROL STOP` before closing the port. The real-time
task consumes that request, resets the controller to IDLE, and applies the
application's zero-command BRAKE behavior; motor shutdown therefore no longer
depends on whether an operating system happens to reset the USB device.

To clear automatically only after a valid CRC and save without plotting:

    capture = ts_capture(port, "capture.mat", true, false)

An optional fifth scalar-structure argument adds application metadata before
the CRC-verified capture is saved and before CLEAR is sent:

    extra.control_config = config;
    capture = ts_capture(port, "capture.mat", true, false, extra)

The USB CDC baud rate argument is a placeholder; the native USB link does not
use UART baud timing.

`ts_capture` validates the CRC before decoding or optionally clearing the
device. It returns both the original `int16` matrix in `capture.raw` and the
scaled engineering values in `capture.values`; invalid samples become `NaN`.
Channel names, units, scales, offsets, sample rate, and time vector come from
the firmware header rather than being fixed in the script.

The decoder and CRC implementation can be checked without hardware or the
instrument-control package:

    octave --quiet tools/octave/test_ts_offline.m

The sensor-independent LUT mathematics and binary format live with the firmware
component under `components/esp_angle_lut/tools/octave`. The `ts_*` functions in
this directory are application or transport functions: they select the
`angle_raw` and `control` channels, detect this motor profile's settled duty
plateaus, manage USB commands, decode this firmware's capture format, and plot
its channels. `ts_load_angle_lut_tools()` locates and adds the component library
to the Octave path.

The application calls `esp_angle_lut_apply()`,
`esp_angle_lut_int16_le_bytes()`, and `esp_angle_lut_plot()` directly. Redundant
`ts_*` wrappers for those functions are intentionally not maintained. A
different project can reuse the component tools and replace only the
capture/profile/transport functions from this directory.
