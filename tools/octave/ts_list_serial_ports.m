function varargout = ts_list_serial_ports (varargin)
  % Application compatibility entry point; implementation lives in the library.
  ts_load_timeseries_tools ();
  [varargout{1:nargout}] = esp_ts_ports (varargin{:});
endfunction
