# ESP Time-Series USB Transport

ASCII command and binary dump transport for the time-series recorder over the
ESP32-S3 native USB Serial/JTAG CDC port. The console must remain on UART0 so no
logs are inserted into the binary stream.

Commands are newline terminated:

- PING
- INFO or STATUS
- ARM followed by a sample rate in Hz
- DUMP
- CLEAR
- HELP

`INFO` and `STATUS` return one `OK` line with the state, rates, channel count,
sample count, capacity, and capture identifier. `ARM` is accepted only while
the recorder is `EMPTY`; its rate must divide the producer rate exactly. In
this application, examples include 1000, 500, 250, 200, 100, and 50 Hz.

DUMP is accepted only in FULL. It sends an ASCII TSRECORDER/1 header, ending in
END-HEADER plus LF, immediately followed by exactly payload_bytes of
sample-interleaved, little-endian int16 data. payload_crc32 is CRC-32/IEEE of
the complete binary payload. A failed or disconnected transfer does not clear
the recorder; the host can request DUMP again.

The two board connectors have separate roles:

- the USB-UART/programming connector carries flashing, console, and logs;
- the ESP32-S3 native USB connector carries this recorder protocol.

Keeping those roles separate is part of the protocol: opening a console on the
native port would insert text into the binary payload, so the component rejects
builds configured to use USB Serial/JTAG as a console.

USB baud rate is ignored by the hardware CDC device. Host software may use
115200 as a conventional placeholder.
