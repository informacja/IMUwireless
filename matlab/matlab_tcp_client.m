%% Polaczenie z ESP32-C3 przez TCP i odczyt strumienia danych w czasie rzeczywistym
% Wymaga MATLAB R2019b lub nowszego (funkcja tcpclient jest wbudowana,
% nie potrzeba dodatkowego toolboxa).

clear; clc; close all;

esp32IP   = "192.168.100.134";   % <-- wpisz adres IP wypisany w Serial Monitorze ESP32
esp32Port = 3333;

disp("Laczenie z ESP32...");
t = tcpclient(esp32IP, esp32Port, "Timeout", 5);
configureTerminator(t, "LF");   % dane koncza sie znakiem \n
flush(t);
disp("Polaczono.");

% Bufory na dane do wykresu na zywo
maxPunktow = 500;
czas  = nan(1, maxPunktow);
wart1 = nan(1, maxPunktow);
wart2 = nan(1, maxPunktow);

figure("Name", "Dane z ESP32-C3 (na zywo)");
h1 = plot(nan, nan, "-b", "DisplayName", "Wartosc 1");
hold on;
h2 = plot(nan, nan, "-r", "DisplayName", "Wartosc 2");
legend show;
xlabel("Czas [ms]"); ylabel("Wartosc");
grid on;

idx = 0;
t0 = [];

% Petla dziala dopoki nie zamkniesz okna wykresu
while ishandle(h1)

    if t.NumBytesAvailable > 0
        linia = readline(t);
        dane = sscanf(linia, "%lu,%f,%f");

        if numel(dane) == 3
            idx = idx + 1;
            if isempty(t0)
                t0 = dane(1);
            end

            % przesuniecie bufora w prawo, jesli sie zapelnil
            if idx > maxPunktow
                czas  = [czas(2:end), NaN];
                wart1 = [wart1(2:end), NaN];
                wart2 = [wart2(2:end), NaN];
                idx = maxPunktow;
            end

            czas(idx)  = double(dane(1) - t0);
            wart1(idx) = dane(2);
            wart2(idx) = dane(3);

            set(h1, "XData", czas(1:idx), "YData", wart1(1:idx));
            set(h2, "XData", czas(1:idx), "YData", wart2(1:idx));
            drawnow limitrate;
        end
    end

    % Przyklad wysylania komendy do ESP32 (odkomentuj w razie potrzeby)
    % write(t, sprintf("STATUS\n"));

end

clear t;
disp("Rozlaczono.");
