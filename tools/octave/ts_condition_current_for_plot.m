function [conditioned, fault_mask] = ts_condition_current_for_plot (current, ...
                                                                    raw, ...
                                                                    saturation_count, ...
                                                                    status)
  if (nargin < 1 || nargin > 4)
    print_usage ();
  endif
  if (nargin < 2)
    raw = [];
  endif
  if (nargin < 3)
    saturation_count = 0;
  endif
  if (nargin < 4 || isempty (status))
    status = empty_status (numel (current));
  endif

  current = current(:).';
  nonobservable_mask = status.nonobservable;
  fault_mask = status.fault | (! isfinite (current) & ! nonobservable_mask);
  if (saturation_count > 0 && ! isempty (raw))
    raw = int16 (raw(:).');
    if (numel (raw) != numel (current))
      error ("Raw current and scaled current have different lengths");
    endif
    fault_mask |= (raw == int16 (32767) | raw == int16 (-32767)) ...
                  & ! status.tagged;
  endif

  conditioned = current;
  conditioned(status.tagged) = NaN;
  valid_mask = ! fault_mask & ! nonobservable_mask & isfinite (conditioned);
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

function status = empty_status (count)
  empty = false (1, count);
  status = struct ("brake", empty, "coast", empty, ...
                   "fault_r", empty, "fault_l", empty, ...
                   "fault_both", empty, "unavailable", empty, ...
                   "fault", empty, "nonobservable", empty, ...
                   "tagged", empty, "valid", ! empty);
endfunction
