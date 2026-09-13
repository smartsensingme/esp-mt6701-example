# Extraction baseline and release gates

Baseline application: 02b451094e7098c4221474c0b7f1d1e4aa2df54b.
The tracked tree was clean before extraction; unrelated generated EPS/CSV were
left untracked. This existing commit is the rollback/reference point.

Historical evidence supplied by the user:

- Windows block transfer worked; binary attempts failed.
- macOS binary worked; changing to blocks made download slower.
- A much earlier capture reported 131070 bytes / 0.153 s = 834.4 KiB/s.
  It was not a controlled benchmark of this baseline; do not use it as a
  performance guarantee or directly comparable acceptance threshold.
- Exact Windows, Octave and instrument-control versions for those historical
  trials were not recorded. Linux was not confirmed on hardware.

Local validation uses a simulated serial object to exercise actual receiver
code, including >512-byte data, both modes, one corrupted block followed by a
successful retry, bad global CRC, wrong trailer, incomplete data, invalid
dimensions and repeated stale blocks. No failure sends CLEAR.

Local extraction checks (2026-09-13): reusable-client tests passed, consuming
application `tools/octave/test_ts_offline.m` passed, and ESP-IDF 6.0 firmware
build passed. These results do not constitute physical USB validation.

Before standalone release, record on each physical host:

1. OS, Octave, instrument-control and ESP-IDF versions and firmware Git hash.
2. Same channel layout, rate, payload size, board and cable as the baseline.
3. At least three FULL downloads per mode under test. Record acquisition time,
   transfer seconds, useful-payload KiB/s, CRC result and retry behavior.
4. Verify MAT contents and compare plots in the consuming application.
5. Verify failure leaves the recorder available and motor stop remains the
   application's responsibility.

After extraction, the user reported a successful physical capture on
2026-09-13 and authorized the next stage. OS, transport mode, software versions,
and transfer timing were not supplied with that confirmation. This validates
the reported capture, not both operating systems or equal throughput.

The owner supplied https://github.com/smartsensingme/esp_timeseries_octave.git
and selected MIT on 2026-09-13. A read-only remote check confirmed an empty
repository. Local Git integration may be prepared before publication, but the
independent commit must be pushed before the consuming project's gitlink.
