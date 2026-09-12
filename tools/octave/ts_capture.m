function capture = ts_capture (endpoint, output_file, clear_after, make_plot, ...
                               additional_fields)
  if (nargin < 1 || nargin > 5)
    print_usage ();
  endif
  if (nargin < 2)
    output_file = "";
  endif
  if (nargin < 3)
    clear_after = false;
  endif
  if (nargin < 4)
    make_plot = true;
  endif
  if (nargin < 5)
    additional_fields = struct ();
  endif
  if (! isstruct (additional_fields) || numel (additional_fields) != 1)
    error ("additional_fields must be an empty or scalar structure");
  endif

  owns_device = ischar (endpoint);
  if (owns_device)
    device = ts_open_serial (endpoint, 30);
  else
    device = endpoint;
  endif
  unwind_protect
    flush (device);
    transfer_timer = tic ();
    write (device, uint8 (["DUMP", char(10)]), "uint8");

    [metadata, header_lines] = read_header (device);
    payload_bytes = required_number (metadata, "payload_bytes");
    payload = read_exact (device, payload_bytes);
    transfer_seconds = toc (transfer_timer);

    expected_crc = uint32 (hex2dec (required_value (metadata, ...
                                                   "payload_crc32")));
    actual_crc = ts_crc32_ieee (payload);
    if (actual_crc != expected_crc)
      error ("CRC mismatch: received %08X, calculated %08X", ...
             expected_crc, actual_crc);
    endif

    channel_count = required_number (metadata, "channel_count");
    sample_count = required_number (metadata, "sample_count");
    sample_rate_hz = required_number (metadata, "sample_rate_hz");
    invalid_i16 = int16 (required_number (metadata, "invalid_i16"));
    if (payload_bytes != 2 * channel_count * sample_count)
      error ("Header dimensions do not match payload_bytes");
    endif

    channels = repmat (struct ("name", "", "unit", "", "scale", 0, ...
                               "offset", 0, "saturation_count", 0, ...
                               "invalid_count", 0, "encoding", "linear"), ...
                       channel_count, 1);
    scales = zeros (channel_count, 1);
    offsets = zeros (channel_count, 1);
    for channel = 0:(channel_count - 1)
      prefix = sprintf ("channel.%d.", channel);
      channels(channel + 1).name = required_value (metadata, ...
                                                   [prefix, "name"]);
      channels(channel + 1).unit = required_value (metadata, ...
                                                   [prefix, "unit"]);
      channels(channel + 1).scale = required_number (metadata, ...
                                                      [prefix, "scale"]);
      channels(channel + 1).offset = required_number (metadata, ...
                                                       [prefix, "offset"]);
      channels(channel + 1).saturation_count = required_number ( ...
          metadata, [prefix, "saturation_count"]);
      channels(channel + 1).invalid_count = required_number ( ...
          metadata, [prefix, "invalid_count"]);
      channels(channel + 1).encoding = optional_value ( ...
          metadata, [prefix, "encoding"], "linear");
      scales(channel + 1) = channels(channel + 1).scale;
      offsets(channel + 1) = channels(channel + 1).offset;
    endfor

    [raw, values] = ts_decode_payload (payload, channel_count, sample_count, ...
                                       invalid_i16, scales, offsets);
    current_status = [];
    tagged_current = find (strcmp ({channels.encoding}, ...
                                   "bts7960-current-v1"));
    if (numel (tagged_current) > 1)
      error ("Capture contains more than one tagged BTS7960 current channel");
    elseif (! isempty (tagged_current))
      current_status = ts_decode_current_status (raw(tagged_current, :));
      values(tagged_current, current_status.tagged) = NaN;
    endif
    capture = struct ();
    capture.protocol = "TSRECORDER/1";
    capture.capture_id = required_number (metadata, "capture_id");
    capture.producer_rate_hz = required_number (metadata, ...
                                                "producer_rate_hz");
    capture.sample_rate_hz = sample_rate_hz;
    capture.start_time_us = required_number (metadata, "start_time_us");
    capture.time_s = (0:(sample_count - 1)) / sample_rate_hz;
    capture.channels = channels;
    capture.raw = raw;
    capture.values = values;
    if (! isempty (current_status))
      capture.current_status = current_status;
    endif
    capture.payload_crc32 = actual_crc;
    capture.header_lines = header_lines;
    capture.usb_transfer_seconds = transfer_seconds;
    capture.usb_transfer_kib_s = payload_bytes / 1024 / transfer_seconds;

    extra_names = fieldnames (additional_fields);
    for extra_index = 1:numel (extra_names)
      extra_name = extra_names{extra_index};
      if (isfield (capture, extra_name))
        error ("Additional capture field conflicts with '%s'", extra_name);
      endif
      capture.(extra_name) = additional_fields.(extra_name);
    endfor

    fprintf ("Capture %d: %d samples, %d channels, %.3f s, CRC %08X OK\n", ...
             capture.capture_id, sample_count, channel_count, ...
             sample_count / sample_rate_hz, actual_crc);
    fprintf ("USB DUMP: %d bytes in %.3f s (%.1f KiB/s)\n", ...
             payload_bytes, transfer_seconds, capture.usb_transfer_kib_s);

    if (! isempty (output_file))
      save ("-mat7-binary", output_file, "capture");
      fprintf ("Saved %s\n", output_file);
    endif
    if (make_plot)
      ts_plot_capture (capture);
    endif
    if (clear_after)
      write (device, uint8 (["CLEAR", char(10)]), "uint8");
      response = strtrim (char (readline (device)));
      if (! strncmp (response, "OK command=CLEAR", 16))
        error ("Capture verified, but CLEAR failed: %s", response);
      endif
      fprintf ("%s\n", response);
    else
      fprintf ("Capture remains FULL. Use ts_command(port, \"CLEAR\") when ready.\n");
    endif
  unwind_protect_cleanup
    if (owns_device)
      clear device;
    endif
  end_unwind_protect
endfunction

function [metadata, header_lines] = read_header (device)
  first_line = strtrim (char (readline (device)));
  if (strncmp (first_line, "ERR ", 4))
    error ("Firmware rejected DUMP: %s", first_line);
  endif
  if (! strcmp (first_line, "TSRECORDER/1"))
    error (["Unexpected protocol line: %s. Close every serial monitor using ", ...
            "the native USB port."], first_line);
  endif

  metadata = containers.Map ("KeyType", "char", "ValueType", "char");
  header_lines = {first_line};
  while (true)
    line = strtrim (char (readline (device)));
    header_lines{end + 1} = line;
    if (strcmp (line, "END-HEADER"))
      break;
    endif
    separator = find (line == "=", 1);
    if (isempty (separator) || separator == 1)
      error ("Malformed header line: %s", line);
    endif
    key = line(1:(separator - 1));
    value = line((separator + 1):end);
    metadata(key) = value;
  endwhile
endfunction

function value = required_value (metadata, key)
  if (! isKey (metadata, key))
    error ("Required header key missing: %s", key);
  endif
  value = metadata(key);
endfunction

function value = optional_value (metadata, key, default_value)
  if (isKey (metadata, key))
    value = metadata(key);
  else
    value = default_value;
  endif
endfunction

function value = required_number (metadata, key)
  text = required_value (metadata, key);
  value = str2double (text);
  if (! isfinite (value))
    error ("Header key %s is not numeric: %s", key, text);
  endif
endfunction

function payload = read_exact (device, byte_count)
  payload = zeros (byte_count, 1, "uint8");
  offset = 1;
  while (offset <= byte_count)
    chunk = read (device, byte_count - offset + 1, "uint8");
    if (isempty (chunk))
      error ("USB timeout after %d of %d payload bytes", offset - 1, ...
             byte_count);
    endif
    chunk = uint8 (chunk(:));
    last = offset + numel (chunk) - 1;
    payload(offset:last) = chunk;
    offset = last + 1;
  endwhile
endfunction
