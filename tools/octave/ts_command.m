function response = ts_command (port_name, command)
  if (nargin != 2)
    print_usage ();
  endif
  ts_load_instrument_control ();
  device = [];
  unwind_protect
    device = serialport (port_name, 115200);
    set (device, "Timeout", 10);
    configureTerminator (device, "lf");
    pause (0.1);
    flush (device);
    write (device, uint8 ([char(command), char(10)]), "uint8");
    response = strtrim (char (readline (device)));
    fprintf ("%s\n", response);
  unwind_protect_cleanup
    clear device;
  end_unwind_protect
endfunction
