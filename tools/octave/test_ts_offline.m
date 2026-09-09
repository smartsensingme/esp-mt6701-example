addpath (fileparts (mfilename ("fullpath")));

known = ts_crc32_ieee (uint8 ("123456789"));
assert (known == uint32 (hex2dec ("CBF43926")));

raw_expected = int16 ([100, -20, -32768; 300, 400, 500]);
flat = raw_expected(:);
unsigned = mod (double (flat), 65536);
payload = zeros (2 * numel (flat), 1, "uint8");
payload(1:2:end) = uint8 (mod (unsigned, 256));
payload(2:2:end) = uint8 (floor (unsigned / 256));
[raw, values] = ts_decode_payload (payload, 2, 3, int16 (-32768), ...
                                   [0.1; 0.01], [0; 1]);
assert (isequal (raw, raw_expected));
assert (values(1, 1) == 10);
assert (values(2, 1) == 4);
assert (isnan (values(1, 3)));

[conditioned, faults] = ts_condition_current_for_plot ([1, NaN, 3], ...
                                                        int16 ([1000, -32768, 3000]), ...
                                                        0);
assert (isequal (faults, logical ([0, 1, 0])));
assert (isequal (conditioned, [1, 2, 3]));

[conditioned, faults] = ts_condition_current_for_plot ([1, 32.767, 5], ...
                                                        int16 ([1000, 32767, 5000]), ...
                                                        1);
assert (isequal (faults, logical ([0, 1, 0])));
assert (isequal (conditioned, [1, 3, 5]));

fs = 500;
t = (0:(24 * fs - 1)) / fs;
speed_rpm = [613 * ones(1, 8 * fs), 887 * ones(1, 8 * fs), ...
             619 * ones(1, 8 * fs)];
true_turns = cumsum (speed_rpm / 60 / fs);
true_angle = mod (true_turns * 360, 360);
measured_angle = mod (true_angle + 0.8 * sin (2 * true_angle * pi / 180) + ...
                      0.45 * sin (4 * true_angle * pi / 180), 360);
control = [40 * ones(1, 8 * fs), 55 * ones(1, 8 * fs), ...
           40 * ones(1, 8 * fs)];
synthetic = struct ();
synthetic.capture_id = 99;
synthetic.sample_rate_hz = fs;
synthetic.time_s = t;
synthetic.channels = struct ("name", {"angle_raw", "control"}, ...
                             "unit", {"deg", "percent"});
synthetic.values = [measured_angle; control];
calibration = ts_calculate_angle_lut (synthetic, 256);
assert (calibration.speed_reduction_percent > 70);
assert (calibration.max_abs_correction_counts == 1024);
assert (calibration.payload_crc32 == ...
        ts_crc32_ieee (ts_int16_le_bytes (calibration.correction_counts)));
corrected = ts_apply_angle_lut (measured_angle, ...
                                calibration.correction_counts, ...
                                calibration.full_scale_counts);
phase_error = mod (corrected - true_angle + 180, 360) - 180;
assert (sqrt (mean (phase_error .^ 2)) < 0.15);

calibration_12_bit = ts_calculate_angle_lut (synthetic, 256, 4096, 256);
assert (calibration_12_bit.full_scale_counts == 4096);
corrected_12_bit = ts_apply_angle_lut (...
    measured_angle, calibration_12_bit.correction_counts, ...
    calibration_12_bit.full_scale_counts);
phase_error_12_bit = mod (corrected_12_bit - true_angle + 180, 360) - 180;
assert (sqrt (mean (phase_error_12_bit .^ 2)) < 0.2);

disp ("Offline Octave recorder tests passed");
