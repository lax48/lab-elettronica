mycomm = serialport('/dev/tty.usbmodem103', 921600);

nvals = 2000;
ndata = 200;

name = "crist-confr-yag-54.2V-buio-sec";

setup_settings(mycomm, ndata, nvals)
pause(1)

valsMatr1 = zeros(nvals, ndata);
valsMatr3 = zeros(nvals, ndata);
mycomm.write('r', 'char');
tic;
for i = 1:nvals
    vals1 = mycomm.read(ndata, 'uint16');
    valsMatr1(i, :) = vals1 / (2^16 - 1) * 3.3;

    vals3 = mycomm.read(ndata, 'uint16');
    valsMatr3(i, :) = vals3 / (2^16 - 1) * 3.3;

    if mod(i, 10) == 0
        disp(['Received data: ' num2str(i)])
    end
end
toc
delete(mycomm)

disp('All data received')

writematrix(valsMatr1, "Dati/" + name + "-ADC1.csv");
writematrix(valsMatr3, "Dati/" + name + "-ADC3.csv");

figure;
hold on;
plot(valsMatr1(1, :))
plot(valsMatr3(1, :))
hold off;
pause(5);

%%%%%%%%
matrice = valsMatr1;

% Crea una figura
figure;

% Ciclo per creare n grafici
hold on; % Per sovrapporre i grafici nella stessa finestra
for i = 1:nvals
    plot(matrice(i, 5:40), '-o', 'DisplayName', ['Riga ' num2str(i)]);
end

disp(mean(valsMatr1 - valsMatr3, "all"))
disp(std(valsMatr1 - valsMatr3, 0, "all"))

% Aggiungi etichette e titolo
xlabel('Indice dei punti');
ylabel('Valori');
title('Grafici delle righe della matrice');
legend show;

% Mostra il grafico
hold off;
drawnow;

%%%%%%%%

pause(5)

maxes1 = max(valsMatr1(:, 5:40), [], 2);
maxes3 = max(valsMatr3(:, 5:40), [], 2);


fig1 = histogram(maxes1, 100);
histValues1 = fig1.Values;
saveas(fig1, "Imgs/" + name + "-ADC1.png")
writematrix(histValues1, "Dati/" + name + "-ADC1.hist.csv")

fig3 = histogram(maxes3, 100);
histValues3 = fig3.Values;
saveas(fig3, "Imgs/" + name + "-ADC3.png")
writematrix(histValues3, "Dati/" + name + "-ADC3.hist.csv")

figure;
hold on;
histogram(maxes1, 100);
fig_comb = histogram(maxes3, 100);
hold off;
drawnow;
saveas(fig_comb, "Imgs/" + name + "-COMB.png")



function setup_settings(mycomm, ndata, nvals)
    if mycomm.NumBytesAvailable > 0
        mycomm.read(mycomm.NumBytesAvailable, "char");
    end

    write(mycomm, 'u', 'char');
    write(mycomm, 24, 'uint32');
    write(mycomm, ndata, 'int16');
    write(mycomm, 350, 'int16');
    write(mycomm, 0, 'uint8');
    write(mycomm, 2, 'uint8');
    write(mycomm, cast(.090/3.3 * (2^16 - 1), 'uint16'), 'uint16');
    write(mycomm, cast(.120/3.3 * (2^16 - 1), 'uint16'), 'uint16');
    write(mycomm, nvals, 'uint16');
end