function response = esp_ts_clear (device)
  % Explicitly discard the capture. Call only after verifying and saving it.
  response = esp_ts_command (device, "CLEAR", false);
  if (! strncmp(response, "OK command=CLEAR ", 17)), error ("%s", response); endif
endfunction
