
%% ============================================================
%  LIVE ORIENTATION - ESP32 + MPU6050 / BNO055
%
%  ESP32 wysyla:
%
%  ax,ay,az,gx,gy,gz
%
%  ax ay az -> przyspieszenie [g]
%  gx gy gz -> predkosc katowa [deg/s]
%
%  Przyklad:
%
%  0.02,-0.01,1.00,0.15,-0.42,1.23
%
%  Filtr komplementarny:
%
%  roll  = alpha * (roll + gx*dt) + (1-alpha)*roll_acc
%  pitch = alpha * (pitch + gy*dt) + (1-alpha)*pitch_acc
%
%  Yaw:
%  tylko integracja gyro -> dryf w czasie
%
% ============================================================

clear;
clc;
close all;


%% ============================================================
% 1. KONFIGURACJA PORTU
% ============================================================

port = "/dev/tty.usbmodem144401";          % <-- ZMIEN NA SWOJ PORT
baudrate = 115200;

% Parametr filtru komplementarnego
%
% alpha blisko 1:
%   wieksze zaufanie do zyroskopu
%
% alpha blisko 0:
%   wieksze zaufanie do akcelerometru
%
alpha = 0.98;

% Maksymalny czas pomiedzy probkami
maxDt = 0.1;


%% ============================================================
% 2. OTWARCIE PORTU
% ============================================================

s = serialport(port, baudrate);

s.Timeout = 1;

flush(s);

disp("==============================================");
disp(" ESP32 IMU LIVE");
disp("==============================================");
disp("Port: " + port);
disp("Baudrate: " + baudrate);
disp("Oczekiwanie na dane...");
disp("");
disp("Format:");
disp("ax,ay,az,gx,gy,gz");
disp("");
disp("Przerwij skrypt Ctrl+C");
disp("==============================================");


%% ============================================================
% 3. INICJALIZACJA ORIENTACJI
% ============================================================

roll  = 0;
pitch = 0;
yaw   = 0;

% Czas
timer = tic;


%% ============================================================
% 4. FIGURA 3D
% ============================================================

fig = figure( ...
    "Name", "ESP32 IMU - LIVE Orientation", ...
    "NumberTitle", "off", ...
    "Color", "w");

ax3d = axes("Parent", fig);

axis(ax3d, "equal");
grid(ax3d, "on");
hold(ax3d, "on");

xlim(ax3d, [-1.5 1.5]);
ylim(ax3d, [-1.5 1.5]);
zlim(ax3d, [-1.5 1.5]);

xlabel(ax3d, "X");
ylabel(ax3d, "Y");
zlabel(ax3d, "Z");

view(ax3d, 3);


%% ============================================================
% 5. BRYLA REPREZENTUJACA IMU
% ============================================================

L = 1.0;
W = 0.5;
H = 0.2;

verts = [ ...
    -L -W -H;
     L -W -H;
     L  W -H;
    -L  W -H;
    -L -W  H;
     L -W  H;
     L  W  H;
    -L  W  H] / 2;

faces = [ ...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];


%% ============================================================
% 6. TRANSFORMACJA BRYLY
% ============================================================

hg = hgtransform("Parent", ax3d);

patch( ...
    "Vertices", verts, ...
    "Faces", faces, ...
    "FaceColor", [0.2 0.6 0.9], ...
    "FaceAlpha", 0.8, ...
    "EdgeColor", "k", ...
    "Parent", hg);


%% ============================================================
% 7. OSIE XYZ
% ============================================================

axisLength = 1.0;

plot3(ax3d, ...
    [0 axisLength], [0 0], [0 0], ...
    "r", "LineWidth", 2);

plot3(ax3d, ...
    [0 0], [0 axisLength], [0 0], ...
    "g", "LineWidth", 2);

plot3(ax3d, ...
    [0 0], [0 0], [0 axisLength], ...
    "b", "LineWidth", 2);


%% ============================================================
% 8. TEKST INFORMACYJNY
% ============================================================

infoText = text( ...
    ax3d, ...
    -1.4, ...
    -1.4, ...
    1.3, ...
    "", ...
    "FontSize", 10, ...
    "VerticalAlignment", "top");


%% ============================================================
% 9. PĘTLA LIVE
% ============================================================

while isvalid(fig)

    % Czy sa dane w buforze?
    if s.NumBytesAvailable > 0

        % -------------------------------------------------------
        % Odczyt jednej linii
        % -------------------------------------------------------

        line = readline(s);

        line = strtrim(line);


        % -------------------------------------------------------
        % Parsowanie danych
        % -------------------------------------------------------

        if strlength(line) == 0
            continue;
        end

        % Wyswietlenie surowej linii w oknie polecen (jak Serial Monitor)
        fprintf("%s\n", line);
        
        pattern = "Accel:\s*X=(-?\d+),\s*Y=(-?\d+),\s*Z=(-?\d+)\s*\|\s*Gyro:\s*X=(-?\d+),\s*Y=(-?\d+),\s*Z=(-?\d+)";

        % Parsowanie liczb z linii
        tokeny = regexp(line, pattern, "tokens", "once");

        if ~isempty(tokeny)
            wartosci = str2double(tokeny);   % [ax ay az gx gy gz]

            % idx = idx + 1;
            % if idx > maxPunktow
            %     buf = [buf(:, 2:end), nan(6, 1)];
            %     idx = maxPunktow;
            % end
            % buf(:, idx) = wartosci(:);
            data = wartosci;
        end
        % data = sscanf(line, "%f,%f,%f,%f,%f,%f");

        % % Musimy otrzymac 6 wartosci
        % if numel(data) ~= 6
        %     continue;
        % end


        %% ------------------------------------------------------
        % Dane IMU
        % -------------------------------------------------------

        ax = data(1);
        ay = data(2);
        az = data(3);

        gx = data(4);
        gy = data(5);
        gz = data(6);


        %% ------------------------------------------------------
        % DT
        % -------------------------------------------------------

        dt = toc(timer);
        timer = tic;

        % Zabezpieczenie przed utrata synchronizacji
        if dt <= 0 || dt > maxDt
            dt = 0.01;
        end


        %% ======================================================
        % 10. ORIENTACJA Z AKCELEROMETRU
        % ======================================================

        %
        % Roll:
        %
        rollAcc = atan2(ay, az);

        %
        % Pitch:
        %
        pitchAcc = atan2( ...
            -ax, ...
            sqrt(ay^2 + az^2));


        %% ======================================================
        % 11. INTEGRACJA ŻYROSKOPU
        % ======================================================

        rollGyro = roll + deg2rad(gx) * dt;

        pitchGyro = pitch + deg2rad(gy) * dt;

        yaw = yaw + deg2rad(gz) * dt;


        %% ======================================================
        % 12. FILTR KOMPLEMENTARNY
        % ======================================================

        roll = ...
            alpha * rollGyro + ...
            (1-alpha) * rollAcc;

        pitch = ...
            alpha * pitchGyro + ...
            (1-alpha) * pitchAcc;


        %% ======================================================
        % 13. TRANSFORMACJA 3D
        % ======================================================

        T = makehgtform( ...
            "zrotate", yaw, ...
            "yrotate", pitch, ...
            "xrotate", roll);

        set(hg, "Matrix", T);


        %% ======================================================
        % 14. AKTUALIZACJA INFORMACJI
        % ======================================================

        % infoText.String = sprintf( ...
        %     [ ...
        %     "ACC [g]\n" ...
        %     "Ax = %7.3f\n" ...
        %     "Ay = %7.3f\n" ...
        %     "Az = %7.3f\n\n" ...
        %     "GYRO [deg/s]\n" ...
        %     "Gx = %7.2f\n" ...
        %     "Gy = %7.2f\n" ...
        %     "Gz = %7.2f\n\n" ...
        %     "ORIENTATION\n" ...
        %     "Roll  = %7.1f deg\n" ...
        %     "Pitch = %7.1f deg\n" ...
        %     "Yaw   = %7.1f deg" ...
        %     ], ...
        %     ax, ay, az, ...
        %     gx, gy, gz, ...
        %     rad2deg(roll), ...
        %     rad2deg(pitch), ...
        %     rad2deg(yaw));


        %% ======================================================
        % 15. TYTUŁ
        % ======================================================

        title(ax3d, ...
            sprintf( ...
            "ESP32 IMU LIVE   |   Roll %.1f°   Pitch %.1f°   Yaw %.1f°", ...
            rad2deg(roll), ...
            rad2deg(pitch), ...
            rad2deg(yaw)));


        %% ======================================================
        % 16. ODŚWIEŻENIE GRAFIKI
        % ======================================================

        drawnow limitrate;

    end

    pause(0.001);

end


%% ============================================================
% 17. ZAMKNIĘCIE
% ============================================================

clear s;

disp("Port szeregowy zamkniety.");
