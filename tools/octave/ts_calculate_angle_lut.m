function calibration = ts_calculate_angle_lut (capture, bin_count, ...
                                                full_scale_counts, ...
                                                max_abs_correction_counts)
  % Adapt this motor application's capture to the reusable LUT calculator.
  if (nargin < 1 || nargin > 4)
    print_usage ();
  endif
  if (nargin < 2)
    bin_count = 256;
  endif
  if (nargin < 3)
    full_scale_counts = 16384;
  endif
  if (nargin < 4)
    max_abs_correction_counts = 1024;
  endif
  ts_load_angle_lut_tools ();

  angle_index = optional_channel (capture, "angle_raw");
  if (isempty (angle_index))
    angle_index = optional_channel (capture, "angle");
  endif
  control_index = optional_channel (capture, "control");
  if (isempty (angle_index) || isempty (control_index))
    error ("Calibration requires angle_raw (or angle) and control channels");
  endif

  time_s = double (capture.time_s(:));
  raw_deg = double (capture.values(angle_index, :)');
  control = double (capture.values(control_index, :)');
  if (numel (time_s) != numel (raw_deg) || numel (time_s) < 100)
    error ("Invalid calibration capture dimensions");
  endif

  plateaus = find_plateaus (time_s, control);
  if (numel (plateaus) < 2)
    error (["At least two steady open-loop plateaus are required. Use the ", ...
            "500 Hz calibration experiment."]);
  endif
  segments = cell (numel (plateaus), 1);
  for index = 1:numel (plateaus)
    segments{index} = plateaus(index).indices;
  endfor

  calibration = esp_angle_lut_calculate ( ...
      time_s, raw_deg, segments, bin_count, full_scale_counts, ...
      max_abs_correction_counts);

  % Attach only application/protocol metadata outside the reusable algorithm.
  calibration.plateaus = plateaus;
  if (isfield (capture, "capture_id"))
    calibration.source_capture_id = capture.capture_id;
  endif
  if (isfield (capture, "sample_rate_hz"))
    calibration.source_sample_rate_hz = capture.sample_rate_hz;
  endif
endfunction

function plateaus = find_plateaus (time_s, control)
  % Application policy: select settled portions of positive-duty stages.
  valid = isfinite (control);
  quantized = round (control * 10) / 10;
  changed = (! valid(2:end)) | (! valid(1:end-1)) | ...
            (abs (diff (quantized)) > 0.2);
  boundary = [true; changed];
  starts = find (boundary);
  ends = [starts(2:end) - 1; numel(control)];
  plateaus = struct ("duty_percent", {}, "start_s", {}, "end_s", {}, ...
                     "indices", {});
  settle_s = 1.5;
  tail_s = 0.5;
  for run = 1:numel (starts)
    first = starts(run);
    last = ends(run);
    duty = median (control(first:last));
    use = find (time_s >= time_s(first) + settle_s & ...
                time_s <= time_s(last) - tail_s & ...
                (1:numel (time_s))' >= first & ...
                (1:numel (time_s))' <= last);
    if (duty > 5 && numel (use) >= 2 && ...
        time_s(use(end)) - time_s(use(1)) >= 3)
      entry = struct ("duty_percent", duty, ...
                      "start_s", time_s(use(1)), ...
                      "end_s", time_s(use(end)), "indices", use);
      plateaus(end + 1) = entry;
    endif
  endfor
endfunction

function index = optional_channel (capture, name)
  names = {capture.channels.name};
  index = find (strcmpi (names, name), 1);
endfunction
