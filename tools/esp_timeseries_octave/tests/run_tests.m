root = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(root,"inst"));
addpath(fullfile(root,"tests"));
assert(esp_ts_crc32(uint8("123456789")) == uint32(hex2dec("CBF43926")));
for mode = {"binary-framed", "block-hex"}
  device = MockSerial();
  capture = esp_ts_capture(device,struct("mode",mode{1}));
  assert(isequal(size(capture.raw),[1,600]));
  assert(strcmp(capture.channels.name,"temperature"));
  assert(capture.payload_crc32 == esp_ts_crc32(device.payload));
  assert(!any(strcmp(device.commands,"CLEAR")));
  assert(device.Timeout == 30);
endfor
device = MockSerial(); device.fault="retry";
capture = esp_ts_capture(device,struct("mode","block-hex"));
assert(device.attempts == 4); % Three blocks plus the corrupted block retry.
for fault = {"short","crc","trailer","dimensions","stale"}
  device = MockSerial(); device.fault=fault{1};
  mode="binary-framed";
  if (strcmp(fault{1},"stale")), mode="block-hex"; endif
  failed=false;
  try
    esp_ts_capture(device,struct("mode",mode));
  catch
    failed=true;
  end_try_catch
  assert(failed);
  assert(device.Timeout == 30);
  assert(!any(strcmp(device.commands,"CLEAR")));
endfor
disp("Reusable Octave transport tests passed");
