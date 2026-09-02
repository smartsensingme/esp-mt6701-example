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

disp ("Offline Octave recorder tests passed");
