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
  angle_index = optional_channel_index (capture, "angle_raw");
  if (isempty (angle_index))
    angle_index = optional_channel_index (capture, "angle");
  endif
  open_loop_capture = ! isempty (angle_index) && all (reference == 0);

  raw_current = [];
  if (isfield (capture, "raw"))
    raw_current = capture.raw(current_index, :);
  endif
  current_status = [];
  if (isfield (capture, "current_status"))
    current_status = capture.current_status;
  endif
  saturation_count = capture.channels(current_index).saturation_count;
  [display_current, current_fault] = ts_condition_current_for_plot ( ...
      current, raw_current, saturation_count, current_status);

  figure_handle = figure ("name", ...
      sprintf ("Captura de controle ESP32 %d", capture.capture_id), ...
      "numbertitle", "off", "units", "normalized", ...
      "position", [0.08, 0.06, 0.84, 0.86]);
  axes_handles = zeros (4, 1);

  if (open_loop_capture)
    axes_handles(1) = subplot (4, 1, 1);
    plot (time_s, speed, "linewidth", 1.1);
    grid on;
    ylabel ("velocidade [rpm]");
    title (sprintf ("Malha aberta - captura %d a %g Hz", ...
                    capture.capture_id, capture.sample_rate_hz));

    axes_handles(2) = subplot (4, 1, 2);
    plot (time_s, capture.values(angle_index, :), "linewidth", 1.0);
    grid on;
    ylabel ("angulo [deg]");
  else
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
  endif

  axes_handles(3) = subplot (4, 1, 3);
  control_handle = plot (time_s, control, "linewidth", 1.1);
  control_handles = control_handle;
  control_labels = {"acao de controle"};
  hold on;
  reverse_braking = isfinite (control) & isfinite (speed) ...
                    & control .* speed < 0;
  if (any (reverse_braking))
    reverse_brake_handle = plot (time_s(reverse_braking), ...
                                 control(reverse_braking), "v", ...
                                 "color", [0.90, 0.45, 0.00], ...
                                 "markersize", 5, ...
                                 "markerfacecolor", [0.90, 0.45, 0.00]);
    control_handles(end + 1) = reverse_brake_handle;
    control_labels{end + 1} = "torque contrario ao movimento";
  endif
  if (! isempty (current_status) && any (current_status.brake))
    brake_handle = plot (time_s(current_status.brake), ...
                         control(current_status.brake), "mo", ...
                         "markersize", 5, "markerfacecolor", "m");
    control_handles(end + 1) = brake_handle;
    control_labels{end + 1} = "freio ativo (BRAKE)";
  endif
  if (numel (control_handles) > 1)
    legend (control_handles, control_labels, "location", "northeast");
  endif
  hold off;
  grid on;
  ylabel ("controle [%]");

  axes_handles(4) = subplot (4, 1, 4);
  current_handles = plot (time_s, display_current, "linewidth", 1.1);
  current_labels = {"corrente"};
  hold on;
  if (! isempty (current_status))
    [current_handles, current_labels] = add_fault_markers ( ...
        current_handles, current_labels, time_s, display_current, ...
        current_status.fault_r, "ro", "falha R_IS");
    [current_handles, current_labels] = add_fault_markers ( ...
        current_handles, current_labels, time_s, display_current, ...
        current_status.fault_l, "rs", "falha L_IS");
    [current_handles, current_labels] = add_fault_markers ( ...
        current_handles, current_labels, time_s, display_current, ...
        current_status.fault_both, "rd", "falha R_IS + L_IS");
    uncategorized_fault = current_fault & ! current_status.fault;
  else
    uncategorized_fault = current_fault;
  endif
  [current_handles, current_labels] = add_fault_markers ( ...
      current_handles, current_labels, time_s, display_current, ...
      uncategorized_fault, "ro", "amostra invalida/saturada");
  if (numel (current_handles) > 1)
    legend (current_handles, current_labels, "location", "northeast");
  endif
  hold off;
  grid on;
  ylabel ("corrente assinada [A]");
  xlabel ("tempo [s]");

  linkaxes (axes_handles, "x");
  set (findall (figure_handle, "type", "axes"), "fontsize", fontSize);
  set (findall (figure_handle, "type", "text"), "fontsize", fontSize);
endfunction

function [handles, labels] = add_fault_markers (handles, labels, time_s, ...
                                                 current, mask, style, label)
  if (any (mask))
    marker = plot (time_s(mask), current(mask), style, "markersize", 6, ...
                   "markerfacecolor", "r");
    handles(end + 1) = marker;
    labels{end + 1} = label;
  endif
endfunction

function index = channel_index (capture, name)
  index = optional_channel_index (capture, name);
  if (isempty (index))
    error ("Capture does not contain the required '%s' channel", name);
  endif
endfunction

function index = optional_channel_index (capture, name)
  names = {capture.channels.name};
  index = find (strcmpi (names, name), 1);
endfunction
