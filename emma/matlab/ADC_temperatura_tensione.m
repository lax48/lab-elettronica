porta = "COM10";

serial = serialport(porta, 115200);

c = "c";
write(serial, c, "char");
calibrazione = read(serial, 3, 'uint16'); % T1, T2 e VREF

% aspetta pressione pulsante

valoriV = read(serial, 10, 'uint16');
valoriT = read(serial, 10, 'uint16');
    
tensione_ref = calibrazione(3)*3.3 ./ valoriV;
temperatura = 30 + (valoriT-calibrazione(1))*(110 - 30) ./ (calibrazione(2)-calibrazione(1));

hold on;
plot(temperatura)
plot(tensione_ref)
hold off;

delete(serial);