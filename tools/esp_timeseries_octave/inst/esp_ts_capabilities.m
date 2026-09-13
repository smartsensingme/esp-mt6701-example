function result = esp_ts_capabilities (device)
  % Inspect HELP, not a guessed firmware version, before selecting a transfer.
  response = esp_ts_command (device, "HELP", false);
  if (! strncmp (response, "OK command=HELP ", 16))
    error ("Firmware HELP failed: %s", response);
  endif
  token = regexp (response, "commands=([^ ]+)", "tokens", "once");
  if (isempty (token)), error ("HELP has no command list"); endif
  result = struct ("commands", {strsplit(token{1}, ",")}, "response", response);
endfunction
