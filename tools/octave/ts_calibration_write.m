function status = ts_calibration_write (endpoint, calibration, enable_after)
  if (nargin < 2 || nargin > 3)
    print_usage ();
  endif
  if (nargin < 3)
    enable_after = false;
  endif
  required = {"bin_count", "correction_counts", "payload_crc32"};
  for index = 1:numel (required)
    if (! isfield (calibration, required{index}))
      error ("Calibration is missing field %s", required{index});
    endif
  endfor
  payload = ts_int16_le_bytes (calibration.correction_counts);
  crc = ts_crc32_ieee (payload);
  if (crc != calibration.payload_crc32)
    error ("Calibration CRC does not match its correction table");
  endif

  owns_device = ischar (endpoint);
  if (owns_device)
    device = ts_open_serial (endpoint, 10);
  else
    device = endpoint;
  endif
  unwind_protect
    command = sprintf ("CAL WRITE %d %08X", calibration.bin_count, crc);
    write (device, uint8 ([command, char(10)]), "uint8");
    ready = strtrim (char (readline (device)));
    if (! strncmp (ready, "OK command=CAL_WRITE state=READY ", 33))
      error ("Firmware did not accept the LUT header: %s", ready);
    endif
    write (device, payload, "uint8");
    response = strtrim (char (readline (device)));
    if (! strncmp (response, "OK command=CAL_WRITE ", 21))
      error ("Firmware rejected the LUT payload: %s", response);
    endif

    readback = ts_calibration_read (device);
    if (! isequal (readback.correction_counts(:), ...
                   int16 (calibration.correction_counts(:))) || ...
        readback.payload_crc32 != crc)
      error ("Calibration readback differs from the uploaded LUT");
    endif
    if (enable_after)
      response = ts_command (device, "CAL ENABLE", false);
      if (! strncmp (response, "OK command=CAL_ENABLE ", 22))
        error ("LUT verified, but enabling failed: %s", response);
      endif
    endif
    status = ts_calibration_status (device, false);
  unwind_protect_cleanup
    if (owns_device)
      clear device;
    endif
  end_unwind_protect
endfunction
