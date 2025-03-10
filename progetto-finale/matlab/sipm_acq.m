mycomm = serialport('/dev/tty.usbmodem103', 921600);

nvals = 50;
ndata = 20;

name = "01-10khz";

setup_settings(mycomm, ndata, nvals)
pause(1)

valsMatr = zeros(nvals, ndata);
mycomm.write('r', 'char');
for i = 1:nvals
    vals = mycomm.read(ndata, 'uint16');
    valsMatr(i, :) = vals / (2^16 - 1) * 3.3;
    disp('Received data')
end
delete(mycomm)

disp('All data received')

writematrix(valsMatr, "Dati/" + name + ".csv");


%%%%%%%%
matrice = valsMatr;

% Crea una figura
figure;

% Ciclo per creare n grafici
hold on; % Per sovrapporre i grafici nella stessa finestra
for i = 1:nvals
    plot(1:ndata, matrice(i, :), '-o', 'DisplayName', ['Riga ' num2str(i)]);
end

% Aggiungi etichette e titolo
xlabel('Indice dei punti');
ylabel('Valori');
title('Grafici delle righe della matrice');
legend show;

yline(.15)
xline(10)

% Mostra il grafico
hold off;
drawnow;

%%%%%%%%

pause(5)

maxes = max(valsMatr, [], 2);
fig = histogram(maxes, 500, "BinLimits", [0, 1.2]);
drawnow;

histValues = fig.Values;

saveas(fig, "Imgs/" + name + ".png")
writematrix(histValues, "Dati/" + name + ".hist.csv")


function setup_settings(mycomm, ndata, nvals)
    if mycomm.NumBytesAvailable > 0
        mycomm.read(mycomm.NumBytesAvailable, "char");
    end

    write(mycomm, 'u', 'char');
    write(mycomm, 24, 'uint32');
    write(mycomm, ndata, 'int16');
    write(mycomm, 5, 'int16');
    write(mycomm, 0, 'uint8');
    write(mycomm, 2, 'uint8');
    write(mycomm, cast(.12/3.3 * (2^16 - 1), 'uint16'), 'uint16');
    write(mycomm, cast(.15/3.3 * (2^16 - 1), 'uint16'), 'uint16');
    write(mycomm, nvals, 'uint16');
end