function varargout = ts_crc32_ieee (varargin)
  % Application compatibility entry point; implementation lives in the library.
  ts_load_timeseries_tools ();
  [varargout{1:nargout}] = esp_ts_crc32 (varargin{:});
endfunction
