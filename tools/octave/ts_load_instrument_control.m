function varargout = ts_load_instrument_control (varargin)
  % Application compatibility entry point; implementation lives in the library.
  ts_load_timeseries_tools ();
  [varargout{1:nargout}] = esp_ts_load_instrument_control (varargin{:});
endfunction
