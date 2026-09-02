function payload = ts_int16_le_bytes (values)
  if (nargin != 1)
    print_usage ();
  endif
  values = int16 (values(:));
  unsigned = mod (double (values), 65536);
  payload = zeros (2 * numel (values), 1, "uint8");
  payload(1:2:end) = uint8 (mod (unsigned, 256));
  payload(2:2:end) = uint8 (floor (unsigned / 256));
endfunction
