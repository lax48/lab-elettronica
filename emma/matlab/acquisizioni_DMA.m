porta = "COM10";
serial = serialport(porta, 115200, "Timeout", 1);

N = 100; % Numero di campioni da leggere
P = 20;  % Pretrigger
K = N-P;
Na = 10;

write(serial, 'c', 'char'); % Invia il carattere 'c' per avviare la comunicazione

% Variabili di controllo del ciclo
i = 1;

while true
    if serial.NumBytesAvailable > 0 % Controlla se ci sono dati disponibili
        dati = read(serial, 1000, "uint16"); % Leggi 100 dati (uint16)
        T = read(serial, 10, "uint16")-2; % Leggi i dati che indicano la posizione
        
        if i < Na+1
            % Gestione del riordino in base a T
            if (T(i) - P) > 0
                B = T(i)-P-1;
                primo = dati(B+2:end); % Prima parte del riordino
                secondo = dati(1:B+1);    % Seconda parte del riordino
            elseif (T(i) - P) < 0
                B = T(i)+K-1;
                primo = dati(B+2:end); % Prima parte del riordino
                secondo = dati(1:B+1);   % Seconda parte del riordino
            else
                primo = dati(K:N); % Prima parte del riordino
                secondo = dati(1:K);    % Seconda parte del riordino
            end

            % Unisco le due parti riordinate
            dati_ordinati = [primo, secondo];

            % Conversione dei dati in valori di tensione (0-3.3V)
            conversione = dati_ordinati * 3.3 / 65535;

            % Visualizza i dati riordinati
            plot(conversione);
            xlabel('Indice');
            ylabel('Tensione (V)');
        
            % Contatore per visualizzare ogni 100 iterazioni
            i = i + 1;
        end
    end

    % Aggiungi un piccolo delay per evitare sovraccarico della CPU
    pause(.01)
end
