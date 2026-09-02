function figure_handle = ts_plot_capture (capture, fontSize)
  if (nargin < 1 || nargin > 2)
    print_usage ();
  endif
  if (nargin < 2)
    fontSize = 12;
  endif
  if (! isscalar (fontSize) || ! isfinite (fontSize) || fontSize <= 0)
    error ("fontSize must be a positive finite scalar");
  endif
  if (! isfield (capture, "channels") || ! isfield (capture, "values") || ...
      ! isfield (capture, "time_s"))
    error ("Invalid capture structure");
  endif

  speed_index = channel_index (capture, "speed");
  current_index = channel_index (capture, "current");
  control_index = channel_index (capture, "control");
  reference_index = channel_index (capture, "reference");
  time_s = capture.time_s;
  speed = capture.values(speed_index, :);
  reference = capture.values(reference_index, :);
  control = capture.values(control_index, :);
  current = capture.values(current_index, :);

  raw_current = [];
  if (isfield (capture, "raw"))
    raw_current = capture.raw(current_index, :);
  endif
  saturation_count = capture.channels(current_index).saturation_count;
  [display_current, current_fault] = ts_condition_current_for_plot ( ...
      current, raw_current, saturation_count);

  figure_handle = figure ("name", ...
      sprintf ("Captura de controle ESP32 %d", capture.capture_id), ...
      "numbertitle", "off", "units", "normalized", ...
      "position", [0.08, 0.06, 0.84, 0.86]);
  axes_handles = zeros (4, 1);

  axes_handles(1) = subplot (4, 1, 1);
  plot (time_s, reference, "--", "linewidth", 1.2, ...
        time_s, speed, "linewidth", 1.1);
  grid on;
  ylabel ("velocidade [rpm]");
  legend ("referencia", "velocidade", "location", "northeast");
  title (sprintf ("Captura %d a %g Hz", capture.capture_id, ...
                  capture.sample_rate_hz));

  axes_handles(2) = subplot (4, 1, 2);
  plot (time_s, reference - speed, "linewidth", 1.1);
  grid on;
  ylabel ("erro [rpm]");

  axes_handles(3) = subplot (4, 1, 3);
  plot (time_s, control, "linewidth", 1.1);
  grid on;
  ylabel ("controle [%]");

  axes_handles(4) = subplot (4, 1, 4);
  plot (time_s, display_current, "linewidth", 1.1);
  hold on;
  if (any (current_fault))
    plot (time_s(current_fault), display_current(current_fault), "ro", ...
          "markersize", 6, "markerfacecolor", "r");
    legend ("corrente", "amostra invalida/saturada", ...
            "location", "northeast");
  endif
  hold off;
  grid on;
  ylabel ("corrente [A]");
  xlabel ("tempo [s]");

  linkaxes (axes_handles, "x");
  set (findall (figure_handle, "type", "axes"), "fontsize", fontSize);
  set (findall (figure_handle, "type", "text"), "fontsize", fontSize);
endfunction

function index = channel_index (capture, name)
  names = {capture.channels.name};
  index = find (strcmpi (names, name), 1);
  if (isempty (index))
    error ("Capture does not contain the required '%s' channel", name);
  endif
endfunction
