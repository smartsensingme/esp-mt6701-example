# Octave receiver

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
`DUMP FRAMED` on `FULL`, validates its terminating record and CRC, saves a MAT
file, and opens one graph
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
duration and KiB/s. Binary payloads are read according to the bytes currently
reported as available by the serial driver, avoiding a blocking request for a
complete final block on Windows. Progress is printed every ten percent and
while waiting for USB data; a timeout therefore reports how many bytes of the
advertised payload were received. If the Windows driver retains the final USB
packet, the receiver queues a `PING`; the firmware's single transport task
answers it only after `END-DUMP`, and that subsequent write releases the pending
bytes without changing the payload. The client consumes these synchronization
responses before issuing `CLEAR`. With
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

    ts_command("/dev/cu.usbmodem1101", "STATUS")
    ts_command("/dev/cu.usbmodem1101", "ARM 250")
    capture = ts_capture("/dev/cu.usbmodem1101", "capture.mat")
    ts_command("/dev/cu.usbmodem1101", "CLEAR")

The application-specific volatile controller commands are:

    ts_command(port, "CONTROL GET")
    ts_command(port, "CONTROL SET 0.25 3.0 0.0001 2.0")
    ts_command(port, "CONTROL DEFAULTS")

The firmware accepts `0 <= Kp <= 100`, `0 <= Ki <= 1000`,
`0 <= Kd <= 10`, and `0.1 <= reference_step_period_s <= 3600`. These bounds
validate the protocol; they are not a claim that every accepted combination is
safe or stable for the connected motor.

This application deliberately gives the host exclusive ownership of the
ARM/CLEAR sequence; there is no application-level automatic pretrigger mode.
Do not leave another serial monitor connected to the native USB port: two
readers split protocol bytes. Use the WCH/USB-UART port for firmware logs.

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
