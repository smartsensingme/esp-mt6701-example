# ESP Time-Series USB Transport

`esp_timeseries_usb_transport` exposes `esp_timeseries_recorder` through the
ESP32-S3 native USB Serial/JTAG CDC port. Commands and metadata are ASCII;
capture samples remain binary for speed. The component knows nothing about the
sensor, motor, control algorithm, or angle calibration.

## Dependencies and installation

The component requires ESP-IDF, a target with the native USB Serial/JTAG
peripheral, and the independent `esp_timeseries_recorder` component. Place both
repositories as sibling components in the application:

```text
project/
├── CMakeLists.txt
├── main/
└── components/
    ├── esp_timeseries_recorder/
    └── esp_timeseries_usb_transport/
```

The recorder is available at
[`smartsensingme/esp_timeseries_recorder`](https://github.com/smartsensingme/esp_timeseries_recorder).
The transport declares this dependency in `CMakeLists.txt`; ESP-IDF resolves it
by component name when both directories are present.

## Hardware and console requirement

On boards with two USB connectors, use the USB-UART/programming connector for
flashing and logs, and the ESP32-S3 native USB connector for this protocol. The
ESP-IDF console must remain on UART: USB console text would corrupt binary DUMP
payloads, and the component rejects that build configuration.

CDC ignores the configured baud rate. A host may still use 115200 as a harmless
API placeholder. The default 16 KiB TX ring and 4 KiB submissions are intended
to keep binary transfer fast; capture time is independently determined by the
recorder capacity and sample rate.

## Configuration and lifecycle

Kconfig controls whether the transport is built, the USB RX/TX driver rings,
and the timeout applied to blocking payload operations. Start it only after the
recorder has been initialized:

```c
ESP_ERROR_CHECK(esp_timeseries_init(&recorder_config));
ESP_ERROR_CHECK(esp_timeseries_usb_transport_start(NULL));
```

`NULL` enables only core commands. `start()` installs the USB driver and creates
a priority-3 task pinned to Core 0. `stop()` deletes that task and uninstalls the
driver. The transport expects exclusive ownership of native USB Serial/JTAG.

## Core commands

Every command ends with LF; CR in CRLF is ignored. Commands are
case-insensitive.

| Command | Purpose | Required recorder state |
|---|---|---|
| `PING` | Check protocol connectivity/version | Any initialized state |
| `INFO` / `STATUS` | Read state, rate, progress, capacity, and capture ID | Any initialized state |
| `ARM <hz>` | Prepare a capture at an exact divisor of producer rate | `EMPTY` |
| `DUMP` | Send header and binary capture without clearing it | `FULL` |
| `CLEAR` | Release the completed capture | `FULL` |
| `HELP` | List core and registered extension commands | Any state |

A typical status line is:

```text
OK command=STATUS protocol=1 state=CAPTURING capture_id=3 producer_rate_hz=1000 sample_rate_hz=500 channels=5 samples=6200 capacity=13107 buffer_bytes=131072
```

Errors use one stable form:

```text
ERR command=ARM code=ESP_ERR_INVALID_ARG message=invalid_sample_rate
```

The recorder rate rules and state machine still apply; the transport does not
silently clear, re-arm, or change a requested rate.

## DUMP framing and reconstruction

`DUMP` begins with `TSRECORDER/1`, followed by newline-separated key/value
metadata and `END-HEADER`. Exactly `payload_bytes` binary bytes follow
immediately. Important fields include:

```text
TSRECORDER/1
capture_id=3
producer_rate_hz=1000
sample_rate_hz=500
sample_count=13107
channel_count=5
encoding=int16
byte_order=little-endian
layout=sample-interleaved
invalid_i16=-32768
channel.0.name=speed
channel.0.unit=rpm
channel.0.scale=0.1
channel.0.offset=0
channel.0.encoding=linear
channel.0.saturation_count=0
channel.0.invalid_count=0
payload_bytes=131070
payload_crc32=1234ABCD
END-HEADER
```

Sample-interleaved means the payload order for three channels is:

```text
s0.c0, s0.c1, s0.c2, s1.c0, s1.c1, s1.c2, ...
```

Each item is a little-endian `int16`. Unless `channel.N.encoding` declares an
application convention, reconstruct it with:

```text
real_value = raw_value * channel.N.scale + channel.N.offset
```

`payload_crc32` is CRC-32/IEEE over exactly the binary payload, not the ASCII
header. The host should reject a length or CRC mismatch. A successful or failed
DUMP leaves the recorder `FULL`, so download may be repeated. Send `CLEAR` only
after the host has safely accepted the capture.

## Application callbacks

An application can coordinate normal ARM and extend the protocol without
putting application dependencies in this reusable component:

```c
static esp_err_t app_arm(uint32_t rate_hz, void *context)
{
    /* Prepare application trigger state, then arm the recorder. */
    return esp_timeseries_arm(rate_hz);
}

static bool app_command(const char *command,
                        const esp_timeseries_usb_command_io_t *io,
                        void *context)
{
    if (strcasecmp(command, "APP STATUS") != 0) {
        return false; /* No I/O before declining the command. */
    }
    (void)io->sendf("OK command=APP_STATUS ready=1\n");
    return true;
}

static const esp_timeseries_usb_transport_config_t usb_config = {
    .arm_handler = app_arm,
    .command_handler = app_command,
    .extension_help = "APP_STATUS",
};

ESP_ERROR_CHECK(esp_timeseries_usb_transport_start(&usb_config));
```

Callbacks execute synchronously in the USB task. They may block that task but
must not block indefinitely. The supplied I/O table is valid only during the
callback: do not save its pointer or delegate its use to another task. The
callback and context pointers, plus `extension_help`, are shallow-copied and
must remain valid until `stop()`.

An extension handler must follow one of two paths:

- recognize the command, send its complete response through `io`, and return
  `true`;
- perform no I/O and return `false`, allowing the core UNKNOWN response.

`write_all` and `read_all` exchange exact binary lengths and retry partial USB
operations. `sendf` is limited to the internal response-line buffer (256 bytes).
`extension_help` is comma-separated text and may not contain CR or LF.

A parent application may, for example, implement a `CAL ...` command family
through this callback. That application module—not this transport—must own the
angle-LUT dependency, validation rules, binary payload, and calibration-capture
coordination.

## Failure behavior and troubleshooting

- `DUMP` before `FULL` returns `capture_not_full`.
- `ARM` while data remain in the recorder returns `recorder_not_empty`; use
  `DUMP` and then `CLEAR` first.
- A stalled/disconnected host causes a timeout and leaves the capture available
  for another `DUMP`.
- Interleaved ESP-IDF log text means the console was assigned to the native USB
  port instead of UART.
- A host reporting “resource busy” usually means another serial monitor or
  process already owns the CDC device.

The host should measure DUMP transfer duration separately from acquisition
duration. At 500 Hz, filling 13,107 records inherently takes about 26.2 seconds
even if the later USB transfer takes only a fraction of a second.
