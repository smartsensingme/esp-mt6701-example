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
That action sends `ARM`, polls `STATUS` every 500 ms, automatically starts
`DUMP` on `FULL`, validates the CRC, saves a MAT file, and opens one graph
with four linked panels: reference plus speed, speed error, control action, and
current. Current samples explicitly marked invalid or saturated by the firmware
are replaced for display by the mean of their valid neighbors and highlighted
with a red circle. The saved raw and scaled data are never modified. Individual
protocol operations remain in an advanced menu.

For an open-loop capture containing the raw `angle` channel, the panels change
to speed, wrapped sensor angle, control duty, and current. The console prints
the predicted/actual acquisition duration separately from the measured USB DUMP
duration and KiB/s. With the current five-channel diagnostic build, select
250 Hz to cover all three 15-second duty stages and the return to COAST.

The plot font size is configurable. Omitting it uses 12 points:

    ts_plot_capture(capture, 16)

The lower-level functions remain available for automated experiments:

    ts_command("/dev/cu.usbmodem1101", "STATUS")
    ts_command("/dev/cu.usbmodem1101", "ARM 250")
    capture = ts_capture("/dev/cu.usbmodem1101", "capture.mat")
    ts_command("/dev/cu.usbmodem1101", "CLEAR")

The default firmware leaves automatic capture disabled so Octave owns the
ARM/CLEAR sequence. It can still be enabled in Kconfig for standalone tests.
Do not leave another serial monitor connected to the native USB port: two
readers split protocol bytes. Use the WCH/USB-UART port for firmware logs.

To clear automatically only after a valid CRC and save without plotting:

    capture = ts_capture(port, "capture.mat", true, false)

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
