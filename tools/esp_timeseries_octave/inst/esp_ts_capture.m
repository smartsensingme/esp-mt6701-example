function capture = esp_ts_capture (endpoint, options)
  % Download an existing FULL capture. Does not arm, plot, save, or clear.
  % options.mode: auto, binary-framed, block-hex. See README for all options.
  if (nargin < 2), options = struct (); endif
  options = esp_ts_options (options);
  owns_device = ischar (endpoint);
  if (owns_device)
    device = esp_ts_open (endpoint, options.timeout_s);
  else
    device = endpoint;
  endif
  original_timeout = get (device, "Timeout");
  unwind_protect
    set (device, "Timeout", options.timeout_s);
    flush (device);
    capabilities = esp_ts_capabilities (device);
    transfer_mode = options.mode;
    if (strcmp (transfer_mode, "auto"))
      if (ispc ()), transfer_mode = "block-hex";
      else, transfer_mode = "binary-framed"; endif
    endif
    if (strcmp (transfer_mode, "block-hex"))
      needed = {"DUMP_BEGIN", "DUMP_BLOCK", "DUMP_END"};
    else
      needed = {"DUMP_FRAMED"};
    endif
    if (! all (ismember (needed, capabilities.commands)))
      error ("Firmware does not advertise the requested transport: %s", transfer_mode);
    endif
    transfer_timer = tic ();
    if (strcmp (transfer_mode, "block-hex"))
      transfer_mode = "block-hex";
      write (device, uint8 (["DUMP BEGIN", char(10)]), "uint8");
    else
      transfer_mode = "binary-framed";
      write (device, uint8 (["DUMP FRAMED", char(10)]), "uint8");
    endif

    [metadata, header_lines] = read_header (device);
    payload_bytes = required_number (metadata, "payload_bytes");
    channel_count = required_number (metadata, "channel_count");
    sample_count = required_number (metadata, "sample_count");
    if (payload_bytes <= 0 || payload_bytes != fix (payload_bytes) || ...
        payload_bytes > options.max_payload_bytes || channel_count < 1 || ...
        channel_count != fix (channel_count) || sample_count < 1 || ...
        sample_count != fix (sample_count) || ...
        payload_bytes != 2 * channel_count * sample_count)
      error ("Invalid capture dimensions or payload exceeds configured limit");
    endif
    if (! strcmp (required_value (metadata, "encoding"), "int16") || ...
        ! strcmp (required_value (metadata, "byte_order"), "little-endian") || ...
        ! strcmp (required_value (metadata, "layout"), "sample-interleaved"))
      error ("Unsupported capture encoding");
    endif
    capture_id = required_number (metadata, "capture_id");
    expected_crc = uint32 (hex2dec (required_value (metadata, ...
                                                   "payload_crc32")));
    if (strcmp (transfer_mode, "block-hex"))
      payload = read_dump_blocks (device, capture_id, payload_bytes, options);
    else
      payload = read_binary_payload (device, payload_bytes);
      validate_dump_trailer (device, metadata, capture_id, expected_crc);
    endif
    transfer_seconds = toc (transfer_timer);

    actual_crc = esp_ts_crc32 (payload);
    if (actual_crc != expected_crc)
      error ("CRC mismatch: received %08X, calculated %08X", ...
             expected_crc, actual_crc);
    endif
    if (strcmp (transfer_mode, "block-hex"))
      finish_dump_blocks (device, capture_id);
    endif

    channel_count = required_number (metadata, "channel_count");
    sample_count = required_number (metadata, "sample_count");
    sample_rate_hz = required_number (metadata, "sample_rate_hz");
    if (sample_rate_hz <= 0), error ("Invalid sample rate"); endif
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

    [raw, values] = esp_ts_decode_payload (payload, channel_count, sample_count, ...
                                       invalid_i16, scales, offsets);
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
    capture.payload_crc32 = actual_crc;
    capture.header_lines = header_lines;
    capture.usb_transfer_seconds = transfer_seconds;
    capture.usb_transfer_kib_s = payload_bytes / 1024 / transfer_seconds;
    capture.usb_transfer_mode = transfer_mode;

    if (! isempty (options.progress))
      options.progress (payload_bytes, payload_bytes);
    endif
  unwind_protect_cleanup
    set (device, "Timeout", original_timeout);
    if (owns_device)
      clear device;
    endif
  end_unwind_protect
endfunction

function payload = read_binary_payload (device, byte_count)
  % Binary is validated on macOS; Linux follows the same default but still
  % requires hardware validation. No hexadecimal expansion or round trips.
  payload = read (device, byte_count, "uint8");
  payload = uint8 (payload(:));
  if (numel (payload) != byte_count)
    error ("USB binary payload has %d bytes; expected %d", ...
           numel (payload), byte_count);
  endif
endfunction

function validate_dump_trailer (device, metadata, capture_id, expected_crc)
  if (! strcmp (optional_value (metadata, "dump_trailer", ""), "END-DUMP"))
    error ("Firmware does not advertise the framed DUMP trailer");
  endif
  expected = sprintf ("END-DUMP capture_id=%d payload_crc32=%08X", ...
                      capture_id, expected_crc);
  received = strtrim (char (readline (device)));
  if (! strcmp (received, expected))
    error ("Invalid DUMP trailer: expected '%s', received '%s'", ...
           expected, received);
  endif
endfunction

function payload = read_dump_blocks (device, capture_id, byte_count, options)
  % Request independently identified blocks. Hex encoding costs bandwidth, but
  % prevents delayed control text from ever being accepted as binary samples.
  block_bytes = 512;
  payload = zeros (byte_count, 1, "uint8");
  original_timeout = get (device, "Timeout");
  unwind_protect
    set (device, "Timeout", 2);
    offset = 0;
    next_progress_percent = 10;
    while (offset < byte_count)
      requested = min (block_bytes, byte_count - offset);
      block = request_dump_block (device, capture_id, offset, requested);
      payload((offset + 1):(offset + requested)) = block;
      offset += requested;
      received_percent = floor (100 * offset / byte_count);
      if (received_percent >= next_progress_percent || offset == byte_count)
        if (! isempty (options.progress))
          options.progress (offset, byte_count);
        endif
        while (next_progress_percent <= received_percent)
          next_progress_percent += 10;
        endwhile
      endif
    endwhile
  unwind_protect_cleanup
    set (device, "Timeout", original_timeout);
  end_unwind_protect
endfunction

function block = request_dump_block (device, capture_id, offset, length)
  command = sprintf ("DUMP BLOCK %d %d %d", capture_id, offset, length);
  write (device, uint8 ([command, char(10)]), "uint8");
  failures = 0;
  response_budget = 20;
  while (failures < 5 && response_budget > 0)
    response_budget -= 1;
    try
      line = strtrim (char (readline (device)));
    catch
      failures += 1;
      write (device, uint8 ([command, char(10)]), "uint8");
      continue;
    end_try_catch
    if (strncmp (line, "ERR ", 4))
      error ("Firmware rejected DUMP BLOCK: %s", line);
    endif
    fields = regexp (line, ["^DATA capture_id=([0-9]+) offset=([0-9]+) ", ...
                            "length=([0-9]+) crc32=([0-9A-Fa-f]{8}) ", ...
                            "hex=([0-9A-Fa-f]*)$"], "tokens", "once");
    if (isempty (fields))
      failures += 1;
      write (device, uint8 ([command, char(10)]), "uint8");
      continue;
    endif
    received_id = str2double (fields{1});
    received_offset = str2double (fields{2});
    received_length = str2double (fields{3});
    if (received_id != capture_id || received_offset != offset)
      % A retry may leave a duplicate response queued. Ignore it; the response
      % for the requested offset follows in the firmware command order.
      continue;
    endif
    encoded = fields{5};
    if (received_length != length || numel (encoded) != 2 * length)
      failures += 1;
      write (device, uint8 ([command, char(10)]), "uint8");
      continue;
    endif
    pairs = reshape (encoded, 2, [])';
    block = uint8 (hex2dec (pairs));
    expected_crc = uint32 (hex2dec (fields{4}));
    if (esp_ts_crc32 (block) != expected_crc)
      failures += 1;
      write (device, uint8 ([command, char(10)]), "uint8");
      continue;
    endif
    return;
  endwhile
  error ("DUMP BLOCK failed after retries at offset %d", offset);
endfunction

function finish_dump_blocks (device, capture_id)
  command = sprintf ("DUMP END %d", capture_id);
  expected = sprintf ("OK command=DUMP_END protocol=1 capture_id=%d", ...
                      capture_id);
  original_timeout = get (device, "Timeout");
  unwind_protect
    set (device, "Timeout", 2);
    write (device, uint8 ([command, char(10)]), "uint8");
    for attempt = 1:10
      try
        response = strtrim (char (readline (device)));
        if (strcmp (response, expected))
          return;
        endif
        if (strncmp (response, "ERR ", 4))
          error ("Firmware rejected DUMP END: %s", response);
        endif
        % Delayed duplicate DATA lines are harmless and are drained here.
      catch err
        if (! isempty (strfind (err.message, "Firmware rejected")))
          rethrow (err);
        endif
      end_try_catch
      write (device, uint8 ([command, char(10)]), "uint8");
    endfor
    error ("DUMP END did not synchronize after retries");
  unwind_protect_cleanup
    set (device, "Timeout", original_timeout);
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
  for header_index = 1:1024
    line = strtrim (char (readline (device)));
    header_lines{end + 1} = line;
    if (strcmp (line, "END-HEADER"))
      return;
    endif
    separator = find (line == "=", 1);
    if (isempty (separator) || separator == 1)
      error ("Malformed header line: %s", line);
    endif
    key = line(1:(separator - 1));
    value = line((separator + 1):end);
    if (isKey (metadata, key)), error ("Duplicate header key: %s", key); endif
    metadata(key) = value;
  endfor
  error ("Header exceeds 1024 lines");
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
