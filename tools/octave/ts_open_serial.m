function varargout = ts_open_serial (varargin)
  % Application compatibility entry point; implementation lives in the library.
  ts_load_timeseries_tools ();
  [varargout{1:nargout}] = esp_ts_open (varargin{:});
endfunction
