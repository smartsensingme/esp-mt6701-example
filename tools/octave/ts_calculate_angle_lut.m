function calibration = ts_calculate_angle_lut (capture, bin_count)
  if (nargin < 1 || nargin > 2)
    print_usage ();
  endif
  if (nargin < 2)
    bin_count = 256;
  endif
  if (! isscalar (bin_count) || bin_count < 16 || ...
      bin_count != fix (bin_count) || bitand (bin_count, bin_count - 1) != 0 || ...
      mod (16384, bin_count) != 0)
    error ("bin_count must be a power of two that divides 16384");
  endif

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

  train_phase = [];
  train_error = [];
  validation_phase = [];
  validation_error = [];
  revolution_count = 0;
  for plateau_index = 1:numel (plateaus)
    indices = plateaus(plateau_index).indices;
    [phase, error_rad, revolution] = revolution_errors ( ...
        time_s(indices), raw_deg(indices));
    if (isempty (phase))
      continue;
    endif
    revolution_count += max (revolution);
    training = mod (revolution, 2) == 1;
    train_phase = [train_phase; phase(training)];
    train_error = [train_error; error_rad(training)];
    validation_phase = [validation_phase; phase(! training)];
    validation_error = [validation_error; error_rad(! training)];
  endfor
  if (numel (train_error) < 4 * bin_count || ...
      numel (validation_error) < 2 * bin_count)
    error ("Not enough complete revolutions for a %d-bin calibration", ...
           bin_count);
  endif

  binned_error = bin_median (train_phase, train_error, bin_count);
  binned_error = fill_circular (binned_error);
  selected_harmonics = min (8, floor (bin_count / 4));
  correction_counts = [];
  while (selected_harmonics >= 1)
    smooth_error = circular_harmonic_smooth (binned_error, ...
                                             selected_harmonics);
    smooth_error -= mean (smooth_error);
    previous = [numel(smooth_error); (1:(numel(smooth_error) - 1))'];
    boundary_error = 0.5 * (smooth_error + smooth_error(previous));
    candidate = int16 (round (boundary_error * 16384 / (2 * pi)));
    candidate = int16 (double (candidate) - round (mean (double (candidate))));
    if (valid_monotonic_lut (candidate))
      correction_counts = candidate;
      break;
    endif
    selected_harmonics -= 1;
  endwhile
  if (isempty (correction_counts))
    error ("Could not produce a monotonic correction LUT");
  endif
  if (max (abs (double (correction_counts))) > 1024)
    error ("Calculated correction exceeds the firmware safety limit");
  endif

  centers_rad = ((0:(bin_count - 1))' + 0.5) * 2 * pi / bin_count;
  correction_rad = double (correction_counts) * 2 * pi / 16384;
  validation_raw = bin_median (validation_phase, validation_error, bin_count);
  validation_raw = fill_circular (validation_raw);
  correction_at_centers = interp_cyclic (correction_rad, centers_rad);
  validation_corrected = wrap_pi (validation_raw - correction_at_centers);
  raw_rms_deg = sqrt (mean (validation_raw .^ 2)) * 180 / pi;
  corrected_rms_deg = sqrt (mean (validation_corrected .^ 2)) * 180 / pi;
  reduction_percent = 100 * (1 - corrected_rms_deg / max (raw_rms_deg, eps));

  corrected_deg = ts_apply_angle_lut (raw_deg, correction_counts);
  raw_speed_energy = 0;
  corrected_speed_energy = 0;
  speed_samples = 0;
  for plateau_index = 1:numel (plateaus)
    indices = plateaus(plateau_index).indices;
    dt = diff (time_s(indices));
    raw_speed = diff (unwrap (raw_deg(indices) * pi / 180)) ./ dt * ...
                60 / (2 * pi);
    corrected_speed = diff (unwrap (corrected_deg(indices) * pi / 180)) ./ ...
                      dt * 60 / (2 * pi);
    raw_residual = detrend (raw_speed);
    corrected_residual = detrend (corrected_speed);
    raw_speed_energy += sum (raw_residual .^ 2);
    corrected_speed_energy += sum (corrected_residual .^ 2);
    speed_samples += numel (raw_residual);
  endfor
  raw_speed_rms_rpm = sqrt (raw_speed_energy / speed_samples);
  corrected_speed_rms_rpm = sqrt (corrected_speed_energy / speed_samples);
  speed_reduction_percent = 100 * (1 - corrected_speed_rms_rpm / ...
                                   max (raw_speed_rms_rpm, eps));

  payload = ts_int16_le_bytes (correction_counts);
  calibration = struct ();
  calibration.format_version = 1;
  calibration.sensor_counts = 16384;
  calibration.bin_count = bin_count;
  calibration.correction_counts = correction_counts(:);
  calibration.correction_deg = double (correction_counts(:)) * 360 / 16384;
  calibration.lut_angle_deg = (0:(bin_count - 1))' * 360 / bin_count;
  calibration.payload = payload;
  calibration.payload_crc32 = ts_crc32_ieee (payload);
  calibration.selected_harmonics = selected_harmonics;
  calibration.revolution_count = revolution_count;
  calibration.plateaus = plateaus;
  calibration.phase_deg = centers_rad * 180 / pi;
  calibration.validation_raw_error_deg = validation_raw * 180 / pi;
  calibration.validation_corrected_error_deg = ...
      validation_corrected * 180 / pi;
  calibration.raw_rms_deg = raw_rms_deg;
  calibration.corrected_rms_deg = corrected_rms_deg;
  calibration.reduction_percent = reduction_percent;
  calibration.raw_speed_rms_rpm = raw_speed_rms_rpm;
  calibration.corrected_speed_rms_rpm = corrected_speed_rms_rpm;
  calibration.speed_reduction_percent = speed_reduction_percent;
  calibration.source_capture_id = capture.capture_id;
  calibration.source_sample_rate_hz = capture.sample_rate_hz;
endfunction

function plateaus = find_plateaus (time_s, control)
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

function [phase, error_rad, revolution] = revolution_errors (time_s, angle_deg)
  radians = unwrap (angle_deg * pi / 180);
  direction = sign (median (diff (radians)));
  if (direction == 0)
    phase = [];
    error_rad = [];
    revolution = [];
    return;
  endif
  progress = direction * radians;
  first_crossing = ceil (progress(1) / (2 * pi));
  last_crossing = floor (progress(end) / (2 * pi));
  levels = (first_crossing:last_crossing)' * 2 * pi;
  if (numel (levels) < 3)
    phase = [];
    error_rad = [];
    revolution = [];
    return;
  endif
  crossing_time = interp1 (progress, time_s, levels, "linear");
  phase = [];
  error_rad = [];
  revolution = [];
  for turn = 1:(numel (crossing_time) - 1)
    selected = find (time_s >= crossing_time(turn) & ...
                     time_s < crossing_time(turn + 1));
    fraction = (time_s(selected) - crossing_time(turn)) / ...
               (crossing_time(turn + 1) - crossing_time(turn));
    ideal = direction * (levels(turn) + 2 * pi * fraction);
    measured = radians(selected);
    phase = [phase; mod(measured, 2 * pi)];
    error_rad = [error_rad; wrap_pi(ideal - measured)];
    revolution = [revolution; turn * ones(numel (selected), 1)];
  endfor
endfunction

function values = bin_median (phase, samples, bin_count)
  bins = min (floor (mod (phase, 2 * pi) * bin_count / (2 * pi)) + 1, ...
              bin_count);
  values = NaN (bin_count, 1);
  for index = 1:bin_count
    selected = samples(bins == index);
    if (! isempty (selected))
      values(index) = median (selected);
    endif
  endfor
endfunction

function filled = fill_circular (values)
  count = numel (values);
  valid = find (isfinite (values));
  if (numel (valid) < count / 4)
    error ("Too few angular bins were observed during calibration");
  endif
  positions = valid - 1;
  query = (0:(count - 1))';
  extended_positions = [positions - count; positions; positions + count];
  extended_values = [values(valid); values(valid); values(valid)];
  filled = interp1 (extended_positions, extended_values, query, "linear");
endfunction

function smooth = circular_harmonic_smooth (values, harmonic_count)
  spectrum = fft (values);
  keep = false (numel (values), 1);
  keep(1:(harmonic_count + 1)) = true;
  keep((end - harmonic_count + 1):end) = true;
  spectrum(! keep) = 0;
  smooth = real (ifft (spectrum));
endfunction

function valid = valid_monotonic_lut (corrections)
  bin_width = 16384 / numel (corrections);
  next = corrections([2:end, 1]);
  steps = bin_width + double (next) - double (corrections);
  valid = all (steps > 0 & steps <= 4 * bin_width);
endfunction

function result = interp_cyclic (values, phase)
  count = numel (values);
  position = mod (phase, 2 * pi) * count / (2 * pi);
  first = floor (position);
  fraction = position - first;
  result = values(first + 1) .* (1 - fraction) + ...
           values(mod(first + 1, count) + 1) .* fraction;
endfunction

function value = wrap_pi (value)
  value = mod (value + pi, 2 * pi) - pi;
endfunction

function index = optional_channel (capture, name)
  names = {capture.channels.name};
  index = find (strcmpi (names, name), 1);
endfunction
