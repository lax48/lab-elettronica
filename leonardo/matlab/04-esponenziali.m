clear;
n = 100;
tau1 = 15;
tau2 = 10;
init1 = 63477;
init2 = 63477;

mycomm = serialport('/dev/tty.usbmodem1103', 921600);

write(mycomm, 's', 'char');
write(mycomm, n, 'uint32');
write(mycomm, tau1, "single");
write(mycomm, tau2, 'single');
write(mycomm, init1, 'uint16');
write(mycomm, init2, 'uint16');

write(mycomm, 'r', 'char');
vals = read(mycomm, n, 'int16');

delete(mycomm)

x = (1:length(vals)) - 1;
y = vals / 2^10;

plot(x, y);
xlabel('x');
ylabel('y');
title('Esponenziali');
grid on;