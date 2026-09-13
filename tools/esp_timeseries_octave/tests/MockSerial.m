classdef MockSerial < handle
  % Deterministic device double: exercises the real receiver without USB.
  properties
    Timeout = 30;
    lines = {};
    commands = {};
    payload = uint8(mod(0:1199, 256))';
    fault = "";
    attempts = 0;
  endproperties
  methods
    function v = get(obj, name), v = obj.(name); endfunction
    function set(obj, name, v), obj.(name) = v; endfunction
    function flush(obj), obj.lines = {}; endfunction
    function line = readline(obj)
      if (isempty(obj.lines)), error("Mock timeout"); endif
      line = obj.lines{1}; obj.lines(1) = [];
    endfunction
    function p = read(obj, n, type)
      p = obj.payload;
      if (strcmp(obj.fault,"short")), p = p(1:end-1); endif
      if (strcmp(obj.fault,"crc")), p(1) = bitxor(p(1),uint8(1)); endif
    endfunction
    function write(obj, bytes, type)
      cmd = strtrim(char(bytes));
      obj.commands{end+1} = cmd;
      crc = esp_ts_crc32(obj.payload);
      if (strcmp(cmd,"HELP"))
        obj.lines{end+1} = "OK command=HELP commands=DUMP_FRAMED,DUMP_BEGIN,DUMP_BLOCK,DUMP_END";
      elseif (any(strcmp(cmd,{"DUMP BEGIN","DUMP FRAMED"})))
        obj.lines = {"TSRECORDER/1", "capture_id=7", "producer_rate_hz=1000", ...
          "sample_rate_hz=500", "sample_count=600", "channel_count=1", ...
          "start_time_us=1000", "encoding=int16", "byte_order=little-endian", ...
          "layout=sample-interleaved", "invalid_i16=-32768", ...
          "channel.0.name=temperature", "channel.0.unit=C", ...
          "channel.0.scale=0.01", "channel.0.offset=0", ...
          "channel.0.saturation_count=0", "channel.0.invalid_count=0", ...
          "payload_bytes=1200", sprintf("payload_crc32=%08X",crc), ...
          "dump_trailer=END-DUMP", "END-HEADER"};
        if (strcmp(obj.fault,"dimensions")), obj.lines{6}="channel_count=0"; endif
        if (strcmp(cmd,"DUMP FRAMED"))
          trailer=sprintf("END-DUMP capture_id=7 payload_crc32=%08X",crc);
          if (strcmp(obj.fault,"trailer")), trailer="OK command=PING protocol=1"; endif
          obj.lines{end+1}=trailer;
        endif
      elseif (strncmp(cmd,"DUMP BLOCK ",11))
        fields=sscanf(cmd,"DUMP BLOCK %d %d %d");
        offset=fields(2); len=fields(3);
        p=obj.payload(offset+(1:len));
        block_crc=esp_ts_crc32(p);
        obj.attempts+=1;
        if (strcmp(obj.fault,"retry") && obj.attempts==1)
          block_crc=bitxor(block_crc,uint32(1));
        endif
        if (strcmp(obj.fault,"stale"))
          offset+=1;
        endif
        obj.lines{end+1}=sprintf("DATA capture_id=7 offset=%d length=%d crc32=%08X hex=%s", ...
                                offset,len,block_crc,sprintf("%02X",p));
      elseif (strcmp(cmd,"DUMP END 7"))
        obj.lines{end+1}="OK command=DUMP_END protocol=1 capture_id=7";
      else
        error("Unexpected command: %s",cmd);
      endif
    endfunction
  endmethods
endclassdef
