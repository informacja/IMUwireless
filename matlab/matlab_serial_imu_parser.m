%% Odczyt i parsowanie danych z portu szeregowego ESP32 (logi w stylu ESP-IDF)
% Oczekiwany format linii, np.:
%   I (383) example: Accel: X=6072, Y=-3700, Z=14832 | Gyro: X=175, Y=214, Z=51
%
% Wymaga MATLAB R2019b+ (funkcja serialport wbudowana, bez toolboxa).

clear; clc; close all;

%% Konfiguracja portu
portName = "/dev/tty.usbmodem144401";   % <-- podmien na swoj port (Windows: "COM5" itp.)
baudRate = 115200;

disp("Laczenie z portem szeregowym...");
s = serialport(portName, baudRate);
configureTerminator(s, "LF");
flush(s);
disp("Polaczono. Nasluchiwanie danych (Ctrl+C w oknie wykresu, aby zakonczyc)...");

% Wyrazenie regularne dopasowujace linie z Accel/Gyro niezaleznie od
% dowolnego prefiksu logu ESP-IDF (np. "I (383) example: ")
pattern = "Accel:\s*X=(-?\d+),\s*Y=(-?\d+),\s*Z=(-?\d+)\s*\|\s*Gyro:\s*X=(-?\d+),\s*Y=(-?\d+),\s*Z=(-?\d+)";

%% Bufory do wykresu na zywo
maxPunktow = 300;
idx = 0;
buf = nan(6, maxPunktow);   % wiersze: ax, ay, az, gx, gy, gz

fig = figure("Name", "Dane IMU z portu szeregowego (na zywo)");
tiledlayout(4, 2, "TileSpacing", "compact");

labels = ["Accel X", "Gyro X", "Accel Y", "Gyro Y", "Accel Z", "Gyro Z"];
hLines = gobjects(1, 6);
for i = 1:6
    nexttile;
    hLines(i) = plot(nan, nan);
    ylabel(labels(i)); grid on;
    if i >= 5, xlabel("probka"); end
end
a = nexttile;
%% Petla glowna - czytanie, wyswietlanie i parsowanie linii
while ishandle(fig)

    if s.NumBytesAvailable > 0
        linia = readline(s);
        linia = strtrim(linia);

        if strlength(linia) == 0
            continue;
        end

        % Wyswietlenie surowej linii w oknie polecen (jak Serial Monitor)
        fprintf("%s\n", linia);

        % Parsowanie liczb z linii
        tokeny = regexp(linia, pattern, "tokens", "once");

        if ~isempty(tokeny)
            wartosci = str2double(tokeny);   % [ax ay az gx gy gz]

            idx = idx + 1;
            if idx > maxPunktow
                buf = [buf(:, 2:end), nan(6, 1)];
                idx = maxPunktow;
            end
            buf(:, idx) = wartosci(:);

            for i = 1:6
                set(hLines(i), "XData", 1:idx, "YData", buf(i, 1:idx));
            end
%% 4. ANIMACJA ORIENTACJI 3D - najbardziej intuicyjna wizualizacja
% Prosta metoda: calkowanie zyroskopu (bez fuzji z akcelerometrem).
% UWAGA: sama integracja gyro dryfuje w czasie - do dokladnej orientacji
% warto polaczyc z akcelerometrem (filtr komplementarny) albo uzyc
% funkcji "ahrsfilter"/"complementaryFilter" z Sensor Fusion and
% Tracking Toolbox, jesli go posiadasz.
gx= buf(end-4);
gy= buf(end-2);
gz= buf(end);
dt = 1;%1/fs;
roll  = 0; pitch = 0; yaw = 0;   % katy Eulera [rad]
roll_hist = zeros(size(buf));
pitch_hist = zeros(size(buf));
yaw_hist = zeros(size(buf));

for k = 1:length(buf)
    roll  = roll  + deg2rad(gx) * dt;
    pitch = pitch + deg2rad(gy) * dt;
    yaw   = yaw   + deg2rad(gz) * dt;
    roll_hist(k) = roll; pitch_hist(k) = pitch; yaw_hist(k) = yaw;
end

% Budowa prostej bryly (prostopadloscianu) reprezentujacej urzadzenie
% figure("Name", "Animacja orientacji IMU");
axes = a;
ax3d = axes; axis equal; grid on; hold on;
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
xlabel("X"); ylabel("Y"); zlabel("Z");
view(3);

L = 1; W = 0.5; H = 0.2; % wymiary bryly
verts = [ -L -W -H;  L -W -H;  L  W -H; -L  W -H; ...
          -L -W  H;  L -W  H;  L  W  H; -L  W  H ] / 2;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

hg = hgtransform("Parent", a);
patch("Vertices", verts, "Faces", faces, "FaceColor", [0.2 0.6 0.9], ...
      "FaceAlpha", 0.8, "Parent", hg);

% Animacja - co kilka probek, zeby nie renderowac setek klatek naraz
% krok = max(1, round(fs/20)); % ~20 klatek/s
k = 300;
    T = makehgtform("zrotate", yaw_hist(k), ...
                     "yrotate", pitch_hist(k), ...
                     "xrotate", roll_hist(k));
    set(hg, "Matrix", T);
    title(sprintf("t = %.2f s   roll=%.0f°  pitch=%.0f°  yaw=%.0f°", ...
        t(k), rad2deg(roll_hist(k)), rad2deg(pitch_hist(k)), rad2deg(yaw_hist(k))));
    drawnow limitrate;


            drawnow limitrate;
        end
        % Linie niepasujace do wzorca (np. inne logi ESP-IDF) sa tylko
        % wyswietlane w konsoli, bez parsowania - dzieki temu skrypt nie
        % "wywraca sie" na komunikatach startowych/diagnostycznych.
    else
        pause(0.01);
    end

end

clear s;
disp("Rozlaczono.");
