function [conditioned, fault_mask] = ts_condition_current_for_plot (current, ...
                                                                    raw, ...
                                                                    saturation_count)
  if (nargin < 1 || nargin > 3)
    print_usage ();
  endif
  if (nargin < 2)
    raw = [];
  endif
  if (nargin < 3)
    saturation_count = 0;
  endif

  current = current(:).';
  fault_mask = ! isfinite (current);
  if (saturation_count > 0 && ! isempty (raw))
    raw = int16 (raw(:).');
    if (numel (raw) != numel (current))
      error ("Raw current and scaled current have different lengths");
    endif
    fault_mask |= raw == int16 (32767) | raw == int16 (-32767);
  endif

  conditioned = current;
  valid_mask = ! fault_mask & isfinite (current);
  for index = find (fault_mask)
    previous = find (valid_mask(1:max(index - 1, 0)), 1, "last");
    next_start = min (index + 1, numel (current));
    following_relative = find (valid_mask(next_start:end), 1, ...
                               "first");
    if (! isempty (following_relative))
      following = next_start + following_relative - 1;
    else
      following = [];
    endif

    if (! isempty (previous) && ! isempty (following))
      conditioned(index) = (current(previous) + current(following)) / 2;
    elseif (! isempty (previous))
      conditioned(index) = current(previous);
    elseif (! isempty (following))
      conditioned(index) = current(following);
    else
      conditioned(index) = 0;
    endif
  endfor
endfunction
