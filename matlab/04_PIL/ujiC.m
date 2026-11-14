delete(serialportfind)
s = serialport("COM5", 115200, "Timeout", 3);
setDTR(s, true); setRTS(s, true);
pause(1); flush(s);

write(s, [uint8([170 85 1]) zeros(1,8,'uint8')], "uint8");   % reset
pause(0.3);
n1 = s.NumBytesAvailable            % harapan: 10
if n1 > 0, read(s, n1, "uint8"); end

pkt = [uint8([170 85 0]) typecast(single(10),'uint8') typecast(single(0),'uint8')];
write(s, pkt, "uint8"); pause(0.3);
n2 = s.NumBytesAvailable            % harapan: 10
if n2 >= 10
    r = read(s, n2, "uint8");
    duty = typecast(uint8(r(3:6)),  'single')    % harapan: sekitar 0.0687
    cyc  = typecast(uint8(r(7:10)), 'uint32')
end