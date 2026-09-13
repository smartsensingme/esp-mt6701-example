function esp_ts_save (filename, capture)
  % Save the complete capture, retaining raw data and protocol metadata.
  % A filesystem error propagates; callers must not clear on that failure.
  save ("-mat7-binary", filename, "capture");
endfunction
