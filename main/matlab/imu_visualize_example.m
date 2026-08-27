%% Intuicyjna wizualizacja danych z IMU (akcelerometr + zyroskop)
% Podmien sekcje "DANE WEJSCIOWE" na swoje realne pomiary
% (np. wczytane z pliku CSV zapisanego przez ESP32 albo strumieniowo z tcpclient).

clear; clc; close all;

%% 1. DANE WEJSCIOWE (przyklad syntetyczny - podmien na swoje)
fs = 100;                    % czestotliwosc probkowania [Hz]
t  = (0:1/fs:5)';            % czas [s]

% Przyklad: powolny obrot + drgania
ax = 0.2*sin(2*pi*0.5*t) + 0.02*randn(size(t));
ay = 0.1*cos(2*pi*0.5*t) + 0.02*randn(size(t));
az = 1.0 + 0.05*sin(2*pi*2*t) + 0.02*randn(size(t));   % ~1g w spoczynku

gx = 10*sin(2*pi*0.3*t);     % [deg/s]
gy = 5*cos(2*pi*0.3*t);
gz = 15*sin(2*pi*0.1*t);

%% 2. WYKRESY CZASOWE - podstawowa diagnostyka
figure("Name", "IMU - wykresy czasowe");
tiledlayout(3, 2, "TileSpacing", "compact");

nexttile; plot(t, ax); ylabel("a_x [g]"); grid on; title("Akcelerometr");
nexttile; plot(t, gx); ylabel("\omega_x [deg/s]"); grid on; title("Zyroskop");
nexttile; plot(t, ay); ylabel("a_y [g]"); grid on;
nexttile; plot(t, gy); ylabel("\omega_y [deg/s]"); grid on;
nexttile; plot(t, az); ylabel("a_z [g]"); xlabel("Czas [s]"); grid on;
nexttile; plot(t, gz); ylabel("\omega_z [deg/s]"); xlabel("Czas [s]"); grid on;

%% 3. MAGNITUDA WEKTORA PRZYSPIESZENIA - wykrywanie uderzen/wibracji
a_mag = sqrt(ax.^2 + ay.^2 + az.^2);

figure("Name", "Magnituda przyspieszenia");
plot(t, a_mag, "LineWidth", 1.2);
yline(1, "--r", "1 g (spoczynek)");
xlabel("Czas [s]"); ylabel("|a| [g]");
title("Magnituda przyspieszenia - latwo widac uderzenia/wibracje");
grid on;

%% 4. ANIMACJA ORIENTACJI 3D - najbardziej intuicyjna wizualizacja
% Prosta metoda: calkowanie zyroskopu (bez fuzji z akcelerometrem).
% UWAGA: sama integracja gyro dryfuje w czasie - do dokladnej orientacji
% warto polaczyc z akcelerometrem (filtr komplementarny) albo uzyc
% funkcji "ahrsfilter"/"complementaryFilter" z Sensor Fusion and
% Tracking Toolbox, jesli go posiadasz.

dt = 1/fs;
roll  = 0; pitch = 0; yaw = 0;   % katy Eulera [rad]
roll_hist = zeros(size(t));
pitch_hist = zeros(size(t));
yaw_hist = zeros(size(t));

for k = 1:length(t)
    roll  = roll  + deg2rad(gx(k)) * dt;
    pitch = pitch + deg2rad(gy(k)) * dt;
    yaw   = yaw   + deg2rad(gz(k)) * dt;
    roll_hist(k) = roll; pitch_hist(k) = pitch; yaw_hist(k) = yaw;
end

% Budowa prostej bryly (prostopadloscianu) reprezentujacej urzadzenie
figure("Name", "Animacja orientacji IMU");
ax3d = axes; axis equal; grid on; hold on;
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
xlabel("X"); ylabel("Y"); zlabel("Z");
view(3);

L = 1; W = 0.5; H = 0.2; % wymiary bryly
verts = [ -L -W -H;  L -W -H;  L  W -H; -L  W -H; ...
          -L -W  H;  L -W  H;  L  W  H; -L  W  H ] / 2;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

hg = hgtransform("Parent", ax3d);
patch("Vertices", verts, "Faces", faces, "FaceColor", [0.2 0.6 0.9], ...
      "FaceAlpha", 0.8, "Parent", hg);

% Animacja - co kilka probek, zeby nie renderowac setek klatek naraz
krok = max(1, round(fs/20)); % ~20 klatek/s
for k = 1:krok:length(t)
    T = makehgtform("zrotate", yaw_hist(k), ...
                     "yrotate", pitch_hist(k), ...
                     "xrotate", roll_hist(k));
    set(hg, "Matrix", T);
    title(sprintf("t = %.2f s   roll=%.0f°  pitch=%.0f°  yaw=%.0f°", ...
        t(k), rad2deg(roll_hist(k)), rad2deg(pitch_hist(k)), rad2deg(yaw_hist(k))));
    drawnow limitrate;
end

disp("Gotowe. Podmien dane syntetyczne w sekcji 1 na wlasne pomiary z ESP32.");
