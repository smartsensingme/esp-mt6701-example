function options = esp_ts_options (supplied)
  % Validate capture options before touching the serial stream.
  options = struct ("mode", "auto", "timeout_s", 30, ...
                    "max_payload_bytes", 16 * 1024 * 1024, "progress", []);
  if (! isstruct (supplied) || numel (supplied) != 1)
    error ("Options must be a scalar structure");
  endif
  names = fieldnames (supplied);
  for k = 1:numel (names)
    if (! isfield (options, names{k})), error ("Unknown option: %s", names{k}); endif
    options.(names{k}) = supplied.(names{k});
  endfor
  if (! ischar (options.mode) || ...
      ! any (strcmp (options.mode, {"auto", "binary-framed", "block-hex"})))
    error ("Invalid transport mode");
  endif
  for name = {"timeout_s", "max_payload_bytes"}
    value = options.(name{1});
    if (! isnumeric (value) || ! isscalar (value) || ! isfinite (value) || value <= 0)
      error ("Invalid option: %s", name{1});
    endif
  endfor
  if (! isempty (options.progress) && ! isa (options.progress, "function_handle"))
    error ("progress must be a function handle or empty");
  endif
endfunction
