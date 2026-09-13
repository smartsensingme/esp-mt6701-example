# Host library maintenance

Read README.md before changes. This library must run with only its inst/
directory on the Octave path and instrument-control installed for live USB.
Do not depend on application ts_ functions or esp_angle_lut.

Preserve raw int16 data, metadata and CRC. Unknown channel encodings are
returned to the application without applying motor-specific interpretation.
Keep plots, menus, PID commands, CONTROL STOP and CAL commands outside this
library. Acquisition, persistence and CLEAR are explicit separate operations.

Never silently downgrade transport after consuming part of a binary response:
the stream may no longer be synchronized. Check advertised capabilities before
DUMP and fail clearly on unsupported modes. Do not insert PING into payload
reception. Bound bad-frame retries and stale-frame loops.

Run tests/run_tests.m for transport changes. It covers both modes, partial data,
CRC, framing, malformed dimensions and stale responses without physical USB.
Also run the consuming application's offline tests. Test macOS binary and
Windows block mode on actual hardware before claiming unchanged throughput.
Linux hardware validation remains outstanding.

The baseline and release gates are in VALIDATION.md. Do not turn observed
Windows symptoms into a certain driver diagnosis. The owner selected MIT and
the public repository https://github.com/smartsensingme/esp_timeseries_octave.git.
Keep fetch URLs portable. Publish library commits before consuming projects'
submodule references; authentication configuration must remain local.
