function capture = ts_capture (endpoint, output_file, clear_after, make_plot, additional_fields)
  % Motor application adapter: interpret BTS7960 tags and select motor plots.
  % Wire protocol, CRC and channel reconstruction belong to esp_ts_capture.
  if (nargin < 2), output_file = ""; endif
  if (nargin < 3), clear_after = false; endif
  if (nargin < 4), make_plot = true; endif
  if (nargin < 5), additional_fields = struct (); endif
  if (! isstruct(additional_fields) || numel(additional_fields) != 1)
    error ("additional_fields must be a scalar structure");
  endif
  ts_load_timeseries_tools ();
  owns_device = ischar(endpoint);
  if (owns_device), device = esp_ts_open(endpoint, 30);
  else, device = endpoint; endif
  unwind_protect
    capture = esp_ts_capture(device, struct("progress", @show_progress));
    tagged = find(strcmp({capture.channels.encoding}, "bts7960-current-v1"));
    if (numel(tagged) > 1), error ("Multiple BTS7960 channels"); endif
    if (! isempty(tagged))
      capture.current_status = ts_decode_current_status(capture.raw(tagged, :));
      capture.values(tagged, capture.current_status.tagged) = NaN;
    endif
    for name = fieldnames(additional_fields)'
      if (isfield(capture, name{1})), error ("Capture field conflict: %s", name{1}); endif
      capture.(name{1}) = additional_fields.(name{1});
    endfor
    fprintf ("\nCapture %d: %d samples, CRC %08X OK\n", ...
             capture.capture_id, columns(capture.raw), capture.payload_crc32);
    fprintf ("USB DUMP (%s): %.3f s (%.1f KiB/s)\n", ...
             capture.usb_transfer_mode, capture.usb_transfer_seconds, capture.usb_transfer_kib_s);
    if (! isempty(output_file))
      esp_ts_save(output_file, capture);
      fprintf ("Saved %s\n", output_file);
    endif
    if (make_plot), ts_plot_capture(capture); endif
    if (clear_after)
      fprintf ("%s\n", esp_ts_clear(device));
    else
      fprintf ("Capture remains FULL until CLEAR.\n");
    endif
  unwind_protect_cleanup
    if (owns_device), clear device; endif
  end_unwind_protect
endfunction

function show_progress (received, total)
  fprintf ("\rDownload: %d/%d bytes (%3d%%)", received, total, floor(100*received/total));
  fflush(stdout);
endfunction
