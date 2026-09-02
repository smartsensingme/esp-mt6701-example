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

`ts_console` is the recommended interactive entry point. It loads
instrument-control, lists the serial ports in a selection menu, displays the
initial recorder status, and provides menus for `STATUS`, `DUMP`, `ARM`,
`CLEAR`, `INFO`, `PING`, and `HELP`. After a valid DUMP, it saves a MAT file and
opens one graph window per recorded channel.

The lower-level functions remain available for automated experiments:

    ts_command("/dev/cu.usbmodem1101", "STATUS")
    ts_command("/dev/cu.usbmodem1101", "ARM 250")
    capture = ts_capture("/dev/cu.usbmodem1101", "capture.mat")
    ts_command("/dev/cu.usbmodem1101", "CLEAR")

The default firmware arms automatically one second before a 600/900 RPM
reference transition. Poll `STATUS` until it reports `state=FULL`, then call
`ts_capture`. To start immediately at a different rate while the recorder is
`EMPTY`, use `ARM 250` (or another exact divisor of 1000).

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
