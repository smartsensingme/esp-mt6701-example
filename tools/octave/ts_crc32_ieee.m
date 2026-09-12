function crc = ts_crc32_ieee (bytes)
  if (nargin != 1)
    print_usage ();
  endif
  ts_load_angle_lut_tools ();
  crc = esp_angle_lut_crc32_ieee (bytes);
endfunction
