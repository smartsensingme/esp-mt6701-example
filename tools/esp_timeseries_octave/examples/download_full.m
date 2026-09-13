function capture = download_full (device, filename, mode)
  % Download an existing FULL buffer using an already-open serial connection.
  % Called by the user; no internal library function calls this example.
  % The caller owns the connection and any actuator start/stop decisions.
  % Example, after adding inst/ and examples/ to the Octave path:
  %   capture = download_full(device, "capture.mat", "auto");
  if (nargin < 3), mode = "auto"; endif

  % Check state without starting a new acquisition or discarding old data.
  status = esp_ts_status (device);
  if (! strcmp (status.state, "FULL"))
    error ("Recorder must be FULL before downloading");
  endif

  % The downloader validates framing and CRC before returning samples.
  capture = esp_ts_capture (device, struct ("mode", mode));
  esp_ts_save (filename, capture);

  % Keep the buffer for inspection or retry. CLEAR is deliberately explicit
  % and remains the caller's choice after checking the saved file.
  fprintf ("Saved %s. Recorder buffer retained.\n", filename);
endfunction
