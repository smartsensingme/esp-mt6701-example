function library_path = ts_load_angle_lut_tools ()
  % Add the reusable esp_angle_lut host library shipped with the component.
  persistent loaded = false;
  application_tools = fileparts (mfilename ("fullpath"));
  project_root = fileparts (fileparts (application_tools));
  library_path = fullfile (project_root, "components", "esp_angle_lut", ...
                           "tools", "octave");
  if (exist (library_path, "dir") != 7)
    error ("esp_angle_lut Octave tools not found at %s", library_path);
  endif
  if (! loaded || exist ("esp_angle_lut_apply", "file") != 2)
    addpath (library_path);
    loaded = true;
  endif
endfunction
