function status = esp_ts_status (device)
  % Return a validated status structure; never interpret boot logs as state.
  response = esp_ts_command (device, "STATUS", false);
  if (! strncmp (response, "OK command=STATUS ", 18))
    error ("Invalid STATUS response: %s", response);
  endif
  status = struct ("response", response);
  for name = {"state", "capture_id", "samples", "capacity", "sample_rate_hz", "producer_rate_hz"}
    token = regexp (response, [name{1}, "=([^ ]+)"], "tokens", "once");
    if (isempty (token)), error ("Missing STATUS field: %s", name{1}); endif
    if (strcmp (name{1}, "state"))
      status.state = token{1};
    else
      value = str2double (token{1});
      if (! isfinite (value) || value < 0 || value != fix(value))
        error ("Invalid STATUS field: %s", name{1});
      endif
      status.(name{1}) = value;
    endif
  endfor
  if (! any (strcmp(status.state, {"EMPTY", "ARMED", "CAPTURING", "FULL"})))
    error ("Unknown recorder state");
  endif
endfunction
