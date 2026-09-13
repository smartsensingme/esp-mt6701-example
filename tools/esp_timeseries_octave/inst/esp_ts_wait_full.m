function status = esp_ts_wait_full (device, poll_s, timeout_s, progress)
  % Poll an already armed recorder; callback receives the validated status.
  % Does not arm, clear, stop actuators, or change sample rate.
  if (nargin < 2), poll_s = 2; endif
  if (nargin < 3), timeout_s = 120; endif
  if (nargin < 4), progress = []; endif
  if (! isscalar(poll_s) || ! isfinite(poll_s) || poll_s <= 0 || ...
      ! isscalar(timeout_s) || ! isfinite(timeout_s) || timeout_s <= 0)
    error ("Polling interval and timeout must be positive seconds");
  endif
  timer = tic ();
  while (toc(timer) < timeout_s)
    status = esp_ts_status (device);
    if (! isempty(progress)), progress(status); endif
    if (strcmp(status.state, "FULL")), return; endif
    if (strcmp(status.state, "EMPTY")), error ("Recorder is EMPTY"); endif
    pause (min(poll_s, max(0, timeout_s - toc(timer))));
  endwhile
  error ("Timed out waiting for FULL; buffer was not cleared");
endfunction
