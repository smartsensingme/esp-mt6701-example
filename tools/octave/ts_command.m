function response = ts_command (endpoint, command, print_response)
  if (nargin < 2 || nargin > 3)
    print_usage ();
  endif
  if (nargin < 3)
    print_response = true;
  endif

  owns_device = ischar (endpoint);
  if (owns_device)
    device = ts_open_serial (endpoint, 10);
  else
    device = endpoint;
  endif
  unwind_protect
    write (device, uint8 ([char(command), char(10)]), "uint8");
    response = strtrim (char (readline (device)));
    if (print_response)
      fprintf ("%s\n", response);
    endif
  unwind_protect_cleanup
    if (owns_device)
      clear device;
    endif
  end_unwind_protect
endfunction
