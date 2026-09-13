# ESP time-series client for GNU Octave

Reusable client for esp_timeseries_usb_transport, maintained in its own
repository. Only inst/ is needed at runtime. No motor driver,
PID, application paths, or angle-LUT package is required.

## Installation and first capture

Clone the library using the public URL:

```sh
git clone https://github.com/smartsensingme/esp_timeseries_octave.git
```

In the motor example project, it is a submodule at
`tools/esp_timeseries_octave`; run `git submodule update --init --recursive`
after cloning or updating that project.

Install instrument-control once using Octave's package manager, then:

```octave
addpath("/path/to/esp_timeseries_octave/inst");
esp_ts_load_instrument_control();
ports = esp_ts_ports();
disp(ports);
device = esp_ts_open(ports{1}, 30); % Select the native USB port deliberately.
unwind_protect
  status = esp_ts_status(device);
  % ARM can start an actuator if the firmware application defines that callback.
  esp_ts_arm(device, 500);
  esp_ts_wait_full(device, 2, 120); % poll period and overall wait, seconds
  capture = esp_ts_capture(device, struct("mode", "auto"));
  esp_ts_save("capture.mat", capture);
  esp_ts_clear(device); % Only reached after successful validation and save.
unwind_protect_cleanup
  clear device;
end_unwind_protect
```

For an existing FULL buffer, omit ARM and wait. Opening USB can reset some
boards; re-read status rather than assuming RAM contents survived. The motor
application adds its own boot synchronization and CONTROL STOP on console exit.
Releasing the serial object does not itself command an actuator to stop.

## API

| Function | Behavior |
|---|---|
| esp_ts_open(port, timeout_s=30) | Load instrument-control, open at nominal 115200, configure LF, wait 1 s and discard startup input |
| esp_ts_ports() | Normalize and list serial ports; accepts an optional list for offline normalization |
| esp_ts_command(device_or_port, command, print_response=true) | One text request/response; caller interprets application commands |
| esp_ts_status(device) | Validated recorder state, IDs, rate and sample counts |
| esp_ts_capabilities(device) | Command list obtained from HELP |
| esp_ts_arm(device, rate_hz) | Explicitly arm; firmware validates supported rate |
| esp_ts_wait_full(device, poll_s=2, timeout_s=120, callback=[]) | Wait for FULL; callback receives status; no implicit ARM/CLEAR |
| esp_ts_capture(device_or_port, options=struct()) | Download and validate a FULL capture, without saving, plotting or clearing |
| esp_ts_save(filename, capture) | Save a MAT file; filesystem errors propagate |
| esp_ts_clear(device) | Explicitly release the recorder buffer |
| esp_ts_decode_payload(...) / esp_ts_crc32(bytes) | Endian-independent linear decoding and CRC-32/IEEE |

Keep one device object open for a session. Functions receiving an existing
object do not close it. Functions accepting a port string release their local
object on completion/error. To close a session, release all references to that
object with clear; this is why there is no misleading close-by-value wrapper.

## Capture options and output

- mode: auto (default), binary-framed or block-hex. Auto uses blocks on Windows
  and binary elsewhere. macOS binary and Windows blocks were validated in the
  original application; Linux hardware performance is unverified.
- timeout_s: positive serial timeout, default 30 seconds. Block exchanges use
  bounded retries with a 2-second per-read timeout. Existing device Timeout is
  restored even when capture fails.
- max_payload_bytes: allocation guard, default 16 MiB.
- progress: optional function handle receiving (received_bytes, total_bytes).
  Binary reports completion; blocks report roughly each ten percent.

HELP must advertise the required transfer commands. Firmware using the old
256-byte response buffer with long application extensions may fail HELP;
the accompanying transport update increases that buffer to 512 bytes.
No mode fallback is attempted after partial reception.

The returned structure preserves raw (int16 channels-by-samples), values
(linearly scaled doubles), channels (name/unit/scale/offset/encoding/counters),
time_s, capture_id, rates, start_time_us, header_lines, payload_crc32,
usb_transfer_mode, usb_transfer_seconds and usb_transfer_kib_s.
The invalid int16 sentinel maps to NaN in values; raw remains intact.
Custom encodings remain the application's responsibility. For example, this
library does not decode BTS7960 fault markers or invoke motor graphs.

CRC validation is compulsory. Never CLEAR after an exception. A failed binary
stream may need connection resynchronization before another attempt.
The library does not certify motor safety, stop on cable removal, or change
firmware control-loop timing.

## Development and release

`examples/download_full.m` is a motor-independent example for an already-open
connection and FULL buffer. It saves the validated capture without clearing it
or issuing actuator commands. Add both `inst/` and `examples/` to use it.

Publish this repository's commit before publishing a consuming project's
submodule reference to it. Do not add an ESP-IDF `idf_component.yml` for this
host-only Octave library: it is not a firmware component.

```sh
octave --quiet tests/run_tests.m
```

See VALIDATION.md for the baseline and hardware checklist. Current installation
uses addpath; pkg install packaging is deliberately deferred.

## License

MIT; see [LICENSE](LICENSE). This applies to this library, not to external
dependencies such as GNU Octave or instrument-control, which retain their own
licenses.
