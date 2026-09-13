function varargout = ts_decode_payload (varargin)
  % Application compatibility entry point; implementation lives in the library.
  ts_load_timeseries_tools ();
  [varargout{1:nargout}] = esp_ts_decode_payload (varargin{:});
endfunction
