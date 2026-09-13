function device = esp_ts_open (port_name, timeout_seconds)
  if (nargin < 1 || nargin > 2)
    print_usage ();
  endif
  if (nargin < 2)
    timeout_seconds = 30;
  endif

  esp_ts_load_instrument_control ();
  device = serialport (port_name, 115200);
  set (device, "Timeout", timeout_seconds);
  configureTerminator (device, "lf");

  % Opening an ESP32-S3 USB Serial/JTAG COM port on Windows can toggle its CDC
  % control lines and restart the target.  Give the bootloader and application
  % transport time to finish before discarding boot text buffered by the host.
  pause (1.0);
  flush (device);
endfunction
