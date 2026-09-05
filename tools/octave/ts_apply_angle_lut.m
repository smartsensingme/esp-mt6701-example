function corrected_deg = ts_apply_angle_lut (raw_deg, correction_counts, ...
                                              full_scale_counts)
  if (nargin < 2 || nargin > 3)
    print_usage ();
  endif
  if (nargin < 3)
    full_scale_counts = 16384;
  endif
  if (! isscalar (full_scale_counts) || ! isfinite (full_scale_counts) || ...
      full_scale_counts <= 0)
    error ("full_scale_counts must be a positive scalar");
  endif
  correction_counts = double (correction_counts(:));
  bin_count = numel (correction_counts);
  if (bin_count < 2)
    error ("The LUT must contain at least two bins");
  endif

  shape = size (raw_deg);
  raw = mod (double (raw_deg(:)), 360);
  position = raw * bin_count / 360;
  first = floor (position);
  fraction = position - first;
  first_index = first + 1;
  second_index = mod (first + 1, bin_count) + 1;
  correction = correction_counts(first_index) .* (1 - fraction) + ...
               correction_counts(second_index) .* fraction;
  corrected_deg = reshape (mod (raw + correction * 360 / ...
                                full_scale_counts, 360), shape);
endfunction
