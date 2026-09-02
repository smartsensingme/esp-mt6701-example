function [raw, values] = ts_decode_payload (payload, channel_count, ...
                                             sample_count, invalid_i16, ...
                                             scales, offsets)
  payload = uint8 (payload(:));
  expected_bytes = 2 * channel_count * sample_count;
  if (numel (payload) != expected_bytes)
    error ("Payload has %d bytes; expected %d", numel (payload), expected_bytes);
  endif

  low = uint16 (payload(1:2:end));
  high = bitshift (uint16 (payload(2:2:end)), 8);
  unsigned_values = bitor (low, high);
  signed_values = double (unsigned_values);
  negative = signed_values >= 32768;
  signed_values(negative) -= 65536;
  raw = reshape (int16 (signed_values), channel_count, sample_count);

  values = zeros (channel_count, sample_count);
  for channel = 1:channel_count
    values(channel, :) = double (raw(channel, :)) * scales(channel) ...
                         + offsets(channel);
    values(channel, raw(channel, :) == invalid_i16) = NaN;
  endfor
endfunction
