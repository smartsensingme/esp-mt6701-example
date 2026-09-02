function figure_handles = ts_plot_capture (capture)
  if (nargin != 1)
    print_usage ();
  endif
  if (! isfield (capture, "channels") || ! isfield (capture, "values") || ...
      ! isfield (capture, "time_s"))
    error ("Invalid capture structure");
  endif

  channel_count = numel (capture.channels);
  if (rows (capture.values) != channel_count)
    error ("Capture channel metadata does not match the value matrix");
  endif

  figure_handles = zeros (channel_count, 1);
  for channel = 1:channel_count
    descriptor = capture.channels(channel);
    window_name = sprintf ("Capture %d - %s", capture.capture_id, ...
                           descriptor.name);
    figure_handles(channel) = figure ("name", window_name, ...
                                      "numbertitle", "off");
    plot (capture.time_s, capture.values(channel, :), "linewidth", 1.1);
    grid on;
    xlabel ("time [s]");
    if (isempty (descriptor.unit))
      ylabel (descriptor.name);
    else
      ylabel (sprintf ("%s [%s]", descriptor.name, descriptor.unit));
    endif
    title (sprintf ("%s - capture %d at %g Hz", descriptor.name, ...
                    capture.capture_id, capture.sample_rate_hz));
  endfor
endfunction
