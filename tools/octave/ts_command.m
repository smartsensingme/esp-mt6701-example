function response = ts_command (port_name, command)
  if (nargin != 2)
    print_usage ();
  endif
  load_instrument_control ();
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

function load_instrument_control ()
  try
    pkg load instrument-control;
  catch
    error (["The Octave instrument-control package is required. ", ...
            "Install it with: pkg install -forge instrument-control"]);
  end_try_catch
  if (exist ("serialport", "file") == 0)
    error ("instrument-control does not provide the serialport API");
  endif
endfunction
