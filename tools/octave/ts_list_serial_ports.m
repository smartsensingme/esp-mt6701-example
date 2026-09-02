function ports = ts_list_serial_ports (listed)
  if (nargin == 0)
    listed = serialportlist ();
  elseif (nargin != 1)
    print_usage ();
  endif

  if (isempty (listed))
    ports = {};
  elseif (iscell (listed))
    ports = cellfun (@char, listed(:), "uniformoutput", false);
  elseif (ischar (listed))
    ports = cellstr (listed);
  else
    ports = cellstr (listed(:));
  endif

  % macOS exposes call-in /dev/tty.* and call-out /dev/cu.* names. Octave is
  % the connection initiator, so prefer the call-out device when it exists.
  for index = 1:numel (ports)
    if (strncmp (ports{index}, "/dev/tty.", 9))
      callout = ["/dev/cu.", ports{index}(10:end)];
      if (exist (callout, "file") == 2)
        ports{index} = callout;
      endif
    endif
  endfor
  ports = unique (ports, "stable");
endfunction
