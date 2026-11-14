delete(serialportfind)
s = serialport("COM5", 115200, "Timeout", 3);
setDTR(s, true); setRTS(s, true);
pause(0.5); flush(s);
write(s, uint8(1:11), "uint8");
pause(0.3);
n = s.NumBytesAvailable
if n > 0, r = read(s, n, "uint8")
else, disp("tidak ada balasan"), end