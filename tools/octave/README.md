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

The console also provides an end-to-end MT6701 angular calibration assistant.
It runs the synchronized 40/55/40 percent open-loop profile at 500 Hz, extracts
complete steady-state revolutions, estimates a 256-bin cyclic correction LUT,
and validates the result on alternating revolutions that were not used to fit
the table. The result window shows the correction and phase error before/after.
The MAT file contains both the original `capture` and calculated `calibration`.

After user confirmation, Octave uploads the signed 14-bit-count corrections as
binary, checks the IEEE CRC32, reads the table back byte-for-byte, and only then
enables it. The firmware stores two versioned NVS slots so an interrupted write
cannot replace the last valid table. New uploads start disabled. Calibration
management in the console can inspect, enable, disable, read back, or erase the
stored LUT.

For an open-loop capture containing the raw `angle_raw` channel (legacy files
named it `angle`), the panels change
to speed, wrapped sensor angle, control duty, and current. The console prints
the predicted/actual acquisition duration separately from the measured USB DUMP
duration and KiB/s. With the current five-channel diagnostic build, select
250 Hz for a long diagnostic trace. The calibration assistant uses 500 Hz and
the calibration firmware profile uses three eight-second duty stages, returning
to COAST before the five-channel 128 KiB buffer becomes full.

Calibration protocol commands are also available directly:

    ts_calibration_status(port)
    installed = ts_calibration_read(port)
    status = ts_calibration_write(port, calibration, true)

The final argument above requests enablement only after CRC and readback
verification. Firmware commands are `CAL STATUS`, `CAL WRITE`, `CAL READ`,
`CAL ENABLE`, `CAL DISABLE`, and `CAL CLEAR`.

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
