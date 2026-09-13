function response = esp_ts_arm (device, rate_hz)
  % Arm the generic recorder. Application callbacks may also start an actuator.
  if (! isscalar(rate_hz) || ! isfinite(rate_hz) || rate_hz <= 0 || rate_hz != fix(rate_hz))
    error ("Rate must be a positive integer in Hz");
  endif
  response = esp_ts_command (device, sprintf("ARM %d", rate_hz), false);
  if (! strncmp(response, "OK command=ARM ", 15)), error ("%s", response); endif
endfunction
