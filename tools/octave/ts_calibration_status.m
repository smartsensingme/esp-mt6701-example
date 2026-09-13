function status = ts_calibration_status (endpoint, print_response)
  if (nargin < 1 || nargin > 2)
    print_usage ();
  endif
  if (nargin < 2)
    print_response = true;
  endif
  ts_load_timeseries_tools ();
  response = esp_ts_command (endpoint, "CAL STATUS", false);
  if (! strncmp (response, "OK command=CAL_STATUS ", 22))
    error ("Firmware rejected CAL STATUS: %s", response);
  endif
  status = struct ("loaded", logical (number_field (response, "loaded")), ...
                   "enabled", logical (number_field (response, "enabled")), ...
                   "format_version", number_field (response, "format"), ...
                   "bin_count", number_field (response, "bins"), ...
                   "full_scale_counts", ...
                       number_field (response, "full_scale_counts"), ...
                   "max_abs_correction_counts", ...
                       number_field (response, ...
                                     "max_abs_correction_counts"), ...
                   "generation", number_field (response, "generation"), ...
                   "payload_crc32", uint32 (hex2dec ( ...
                       text_field (response, "crc32"))), ...
                   "response", response);
  if (print_response)
    fprintf ("%s\n", response);
  endif
endfunction

function value = number_field (response, key)
  value = str2double (text_field (response, key));
  if (! isfinite (value))
    error ("Invalid numeric field %s in: %s", key, response);
  endif
endfunction

function value = text_field (response, key)
  token = regexp (response, ["(?:^| )", key, "=([^ ]+)"], ...
                  "tokens", "once");
  if (isempty (token))
    error ("Missing field %s in: %s", key, response);
  endif
  value = token{1};
endfunction
