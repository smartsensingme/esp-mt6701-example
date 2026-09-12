function status = ts_decode_current_status (raw)
  if (nargin != 1)
    print_usage ();
  endif

  raw = int16 (raw(:).');
  status = struct ();
  status.brake = raw == int16 (32760);
  status.coast = raw == int16 (32761);
  status.fault_r = raw == int16 (32762);
  status.fault_l = raw == int16 (32763);
  status.fault_both = raw == int16 (32764);
  status.unavailable = raw == int16 (32765);
  status.fault = status.fault_r | status.fault_l | status.fault_both;
  status.nonobservable = status.brake | status.coast | status.unavailable;
  status.tagged = status.fault | status.nonobservable;
  status.valid = ! status.tagged;
endfunction
