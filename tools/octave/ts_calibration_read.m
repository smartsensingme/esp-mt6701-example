function calibration = ts_calibration_read (endpoint)
  if (nargin != 1)
    print_usage ();
  endif
  ts_load_timeseries_tools ();
  owns_device = ischar (endpoint);
  if (owns_device)
    device = esp_ts_open (endpoint, 10);
  else
    device = endpoint;
  endif
  unwind_protect
    write (device, uint8 (["CAL READ", char(10)]), "uint8");
    first = strtrim (char (readline (device)));
    if (strncmp (first, "ERR ", 4))
      error ("Firmware rejected CAL READ: %s", first);
    endif
    protocol = regexp (first, "^ANGLELUT/([0-9]+)$", "tokens", "once");
    if (isempty (protocol))
      error ("Unexpected calibration protocol line: %s", first);
    endif
    format_version = str2double (protocol{1});
    if (! any (format_version == [1, 2]))
      error ("Unsupported calibration format: %d", format_version);
    endif
    metadata = containers.Map ("KeyType", "char", "ValueType", "char");
    while (true)
      line = strtrim (char (readline (device)));
      if (strcmp (line, "END-HEADER"))
        break;
      endif
      separator = find (line == "=", 1);
      if (isempty (separator))
        error ("Malformed calibration header: %s", line);
      endif
      metadata(line(1:(separator - 1))) = line((separator + 1):end);
    endwhile
    bins = required_number (metadata, "bins");
    if (format_version >= 2)
      full_scale_counts = required_number (metadata, "full_scale_counts");
    else
      full_scale_counts = 16384;
    endif
    payload_bytes = required_number (metadata, "payload_bytes");
    if (payload_bytes != 2 * bins)
      error ("Calibration header dimensions are inconsistent");
    endif
    payload = read_exact (device, payload_bytes);
    expected_crc = uint32 (hex2dec (required_text (metadata, ...
                                                   "payload_crc32")));
    actual_crc = esp_ts_crc32 (payload);
    if (actual_crc != expected_crc)
      error ("Calibration readback CRC mismatch");
    endif
    unsigned = double (payload(1:2:end)) + ...
               256 * double (payload(2:2:end));
    signed = unsigned;
    signed(signed >= 32768) -= 65536;
    correction_counts = int16 (signed);
    calibration = struct ();
    calibration.format_version = format_version;
    calibration.bin_count = bins;
    calibration.full_scale_counts = full_scale_counts;
    calibration.sensor_counts = full_scale_counts;
    calibration.generation = required_number (metadata, "generation");
    calibration.enabled = logical (required_number (metadata, "enabled"));
    calibration.correction_counts = correction_counts(:);
    calibration.correction_deg = double (correction_counts(:)) * 360 / ...
                                 full_scale_counts;
    calibration.payload = payload;
    calibration.payload_crc32 = actual_crc;
  unwind_protect_cleanup
    if (owns_device)
      clear device;
    endif
  end_unwind_protect
endfunction

function text = required_text (metadata, key)
  if (! isKey (metadata, key))
    error ("Missing calibration header field: %s", key);
  endif
  text = metadata(key);
endfunction

function value = required_number (metadata, key)
  value = str2double (required_text (metadata, key));
  if (! isfinite (value))
    error ("Calibration header field %s is not numeric", key);
  endif
endfunction

function payload = read_exact (device, byte_count)
  payload = zeros (byte_count, 1, "uint8");
  offset = 1;
  while (offset <= byte_count)
    chunk = read (device, byte_count - offset + 1, "uint8");
    if (isempty (chunk))
      error ("USB timeout while reading the calibration LUT");
    endif
    chunk = uint8 (chunk(:));
    last = offset + numel (chunk) - 1;
    payload(offset:last) = chunk;
    offset = last + 1;
  endwhile
endfunction
