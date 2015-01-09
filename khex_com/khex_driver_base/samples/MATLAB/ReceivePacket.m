function ret = ReceivePacket()
persistent packetId buf2

if isempty(packetId)
  packetId = kBotPacket2API('create');
end

ret = [];

if ~isempty(buf2)
  [packet buf2] =  kBotPacket2API('processBuffer',packetId,buf2);
  if ~isempty(packet)
    ret = packet;
    return;
  end
end

buf = SerialDeviceAPI('read',1000,2000);
[packet buf2] = kBotPacket2API('processBuffer',packetId,buf);
if ~isempty(packet)
  ret = packet;
end

