function device = ts_open_serial (port_name, timeout_seconds)
  if (nargin < 1 || nargin > 2)
    print_usage ();
  endif
  if (nargin < 2)
    timeout_seconds = 30;
  endif

  ts_load_instrument_control ();
  device = serialport (port_name, 115200);
  set (device, "Timeout", timeout_seconds);
  configureTerminator (device, "lf");
  pause (0.1);
  flush (device);
endfunction
