function crc = ts_crc32_ieee (bytes)
  % CRC-32/IEEE (poly 0x04C11DB7, reflected 0xEDB88320).
  persistent table;
  if (isempty (table))
    polynomial = uint32 (hex2dec ("EDB88320"));
    table = zeros (256, 1, "uint32");
    for index = 0:255
      value = uint32 (index);
      for bit = 1:8
        if (bitand (value, uint32 (1)))
          value = bitxor (bitshift (value, -1), polynomial);
        else
          value = bitshift (value, -1);
        endif
      endfor
      table(index + 1) = value;
    endfor
  endif

  crc = uint32 (hex2dec ("FFFFFFFF"));
  bytes = uint8 (bytes(:));
  for index = 1:numel (bytes)
    lookup = bitand (bitxor (crc, uint32 (bytes(index))), uint32 (255));
    crc = bitxor (bitshift (crc, -8), table(double (lookup) + 1));
  endfor
  crc = bitxor (crc, uint32 (hex2dec ("FFFFFFFF")));
endfunction
