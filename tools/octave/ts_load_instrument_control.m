function ts_load_instrument_control ()
  try
    pkg load instrument-control;
  catch
    error (["The Octave instrument-control package is required. ", ...
            "Install it with: pkg install -forge instrument-control"]);
  end_try_catch
  if (exist ("serialport", "file") == 0 || ...
      exist ("serialportlist", "file") == 0)
    error ("instrument-control does not provide the serialport API");
  endif
endfunction
