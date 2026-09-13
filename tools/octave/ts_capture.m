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
    write (device, uint8 (["DUMP FRAMED", char(10)]), "uint8");

    [metadata, header_lines] = read_header (device);
    payload_bytes = required_number (metadata, "payload_bytes");
    capture_id = required_number (metadata, "capture_id");
    expected_crc = uint32 (hex2dec (required_value (metadata, ...
                                                   "payload_crc32")));
    [payload, nudge_count] = read_exact (device, payload_bytes);
    validate_dump_trailer (device, metadata, capture_id, expected_crc);
    validate_dump_nudges (device, nudge_count);
    transfer_seconds = toc (transfer_timer);

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
    capture.capture_id = capture_id;
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

function validate_dump_nudges (device, nudge_count)
  % The firmware processes commands in one transport task. Therefore every
  % PING written while DUMP is finishing is answered strictly after the binary
  % payload and END-DUMP trailer, preserving unambiguous stream ordering.
  for index = 1:nudge_count
    response = strtrim (char (readline (device)));
    if (! strncmp (response, "OK command=PING ", 16))
      error ("Invalid post-DUMP synchronization response: %s", response);
    endif
  endfor
endfunction

function validate_dump_trailer (device, metadata, capture_id, expected_crc)
  trailer_kind = optional_value (metadata, "dump_trailer", "");
  if (! strcmp (trailer_kind, "END-DUMP"))
    error (["Firmware does not support framed DUMP. Update and flash the ", ...
            "current firmware before downloading on Windows."]);
  endif

  expected = sprintf ("END-DUMP capture_id=%d payload_crc32=%08X", ...
                      capture_id, expected_crc);
  received = strtrim (char (readline (device)));
  if (! strcmp (received, expected))
    error ("Invalid DUMP trailer: expected '%s', received '%s'", ...
           expected, received);
  endif
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

function [payload, nudge_count] = read_exact (device, byte_count)
  % Read only bytes already reported by the serial back end.  On Windows, a
  % blocking read() for the next complete block may wait indefinitely near the
  % end of a USB transfer even while a shorter final fragment is buffered.
  read_chunk_bytes = 4096;
  progress_step_percent = 10;
  inactivity_timeout_s = double (get (device, "Timeout"));
  if (! isfinite (inactivity_timeout_s) || inactivity_timeout_s <= 0)
    inactivity_timeout_s = 30;
  endif
  payload = zeros (byte_count, 1, "uint8");
  offset = 1;
  nudge_count = 0;
  max_nudges = 5;
  next_progress_percent = progress_step_percent;
  inactivity_timer = tic ();
  wait_report_timer = tic ();
  nudge_timer = tic ();
  while (offset <= byte_count)
    available = floor (double (get (device, "NumBytesAvailable")));
    if (available <= 0)
      if (toc (inactivity_timer) >= inactivity_timeout_s)
        fprintf ("\n");
        error ("USB timeout after %d of %d payload bytes", offset - 1, ...
               byte_count);
      endif
      if (toc (wait_report_timer) >= 2)
        received_percent = floor (100 * (offset - 1) / byte_count);
        fprintf ("\rDownload: %6d/%6d bytes (%3d%%), aguardando USB...", ...
                 offset - 1, byte_count, received_percent);
        fflush (stdout);
        wait_report_timer = tic ();
      endif
      if (nudge_count < max_nudges && toc (nudge_timer) >= 1)
        % A subsequent native-USB write makes the Windows driver release bytes
        % that it occasionally retains at the end of a long transmission.
        % PING is queued behind DUMP by the single firmware transport task.
        write (device, uint8 (["PING", char(10)]), "uint8");
        nudge_count += 1;
        nudge_timer = tic ();
      endif
      pause (0.005);
      continue;
    endif

    remaining = byte_count - offset + 1;
    requested = min ([remaining, read_chunk_bytes, available]);
    chunk = read (device, requested, "uint8");
    if (isempty (chunk))
      pause (0.005);
      continue;
    endif
    chunk = uint8 (chunk(:));
    last = offset + numel (chunk) - 1;
    payload(offset:last) = chunk;
    offset = last + 1;
    inactivity_timer = tic ();

    % Report bounded progress so a slow or interrupted Windows transfer is
    % distinguishable from a frozen Octave process.
    received_percent = floor (100 * (offset - 1) / byte_count);
    if (received_percent >= next_progress_percent || offset > byte_count)
      fprintf ("\rDownload: %6d/%6d bytes (%3d%%)", ...
               offset - 1, byte_count, received_percent);
      fflush (stdout);
      while (next_progress_percent <= received_percent)
        next_progress_percent += progress_step_percent;
      endwhile
    endif
  endwhile
  fprintf ("\n");
endfunction
