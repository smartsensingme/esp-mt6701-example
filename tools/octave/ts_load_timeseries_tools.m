function ts_load_timeseries_tools ()
  % Application-local dependency locator. The library has no project paths.
  root = fileparts (mfilename ("fullpath"));
  addpath (fullfile (root, "..", "esp_timeseries_octave", "inst"));
endfunction
