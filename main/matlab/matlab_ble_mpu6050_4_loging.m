%% Odbior danych z MPU6050 przez BLE (ESP32-C3) w MATLAB
% Wymaga MATLAB R2019a+ z obsluga BLE (funkcja "ble" / "characteristic").

clear; clc; close all;

deviceName = "ESP32_MPU6050";   % nazwa reklamowana przez ESP32 (DEVICE_NAME w main.c)

disp("Skanowanie i laczenie z urzadzeniem BLE...");
b = ble(deviceName);
disp("Polaczono z: " + b.Name);

c = characteristic(b, "00FF", "FF01");   % 16-bitowe UUID serwisu i charakterystyki
subscribe(c, 'notification');

%% Zapis do pliku CSV (opcjonalnie - ustaw na false, zeby wylaczyc)
enableLogging = true;
fid = -1;
logFileName = "";
if enableLogging
    logFileName = "mpu6050_log_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
    fid = fopen(logFileName, "w");
    if fid < 0
        warning("Nie udalo sie otworzyc pliku CSV do zapisu - logowanie wylaczone.");
    else
        fprintf(fid, "timestamp,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");
        disp("Zapisywanie surowych danych do pliku: " + logFileName);
    end
end

% Przygotowanie okna z 6 wykresami
fig = figure("Name", "MPU6050 przez BLE (na zywo)");
tiledlayout(3, 2, "TileSpacing", "compact");

labels = ["a_x [g] (filtr, bez trendu)", "\omega_x [dps] (filtr, bez trendu)", ...
          "a_y [g] (filtr, bez trendu)", "\omega_y [dps] (filtr, bez trendu)", ...
          "a_z [g] (filtr, bez trendu)", "\omega_z [dps] (filtr, bez trendu)"];
hLines = gobjects(1, 6);
for i = 1:6
    nexttile;
    hLines(i) = plot(nan, nan);
    ylabel(labels(i)); grid on;
    if i >= 5, xlabel("probka"); end
end

maxPunktow = 300;
fig.UserData = struct("idx", 0, "buf", nan(6, maxPunktow), "lines", hLines, ...
                       "filtPrev", nan(6, 1));

%% Druga figura - animowany szescian 3D (orientacja urzadzenia)
% Uzywamy filtru komplementarnego (zyroskop + akcelerometr), co daje
% znacznie plynniejszy i mniej dryfujacy obraz niz sama integracja gyro.

figCube = figure("Name", "Orientacja MPU6050 (szescian 3D)");
ax3d = axes("Parent", figCube);
axis(ax3d, "equal"); grid(ax3d, "on"); hold(ax3d, "on");
xlim(ax3d, [-1.5 1.5]); ylim(ax3d, [-1.5 1.5]); zlim(ax3d, [-1.5 1.5]);
xlabel(ax3d, "X"); ylabel(ax3d, "Y"); zlabel(ax3d, "Z");
view(ax3d, 3);

L = 1; W = 0.6; H = 0.2;
verts = [ -L -W -H;  L -W -H;  L  W -H; -L  W -H; ...
          -L -W  H;  L -W  H;  L  W  H; -L  W  H ] / 2;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

hg = hgtransform("Parent", ax3d);
patch("Vertices", verts, "Faces", faces, "FaceColor", [0.2 0.6 0.9], ...
      "FaceAlpha", 0.85, "Parent", hg);

% Stan filtru komplementarnego trzymany w UserData figury (bez "persistent",
% zeby dalo sie latwo zresetowac/debugowac miedzy uruchomieniami)
figCube.UserData = struct("roll", 0, "pitch", 0, "yaw", 0, "lastTime", tic, "hg", hg);

% Callback wywolywany automatycznie przy kazdym powiadomieniu BLE
c.DataAvailableFcn = @(src, ~) onNotification(src, fig, figCube, b, fid);

disp("Odbieranie danych na zywo... zamknij oba okna, aby zakonczyc.");
try
    while ishandle(fig) && ishandle(figCube)
        pause(0.05);
    end
catch err
    if fid > 0, fclose(fid); end
    unsubscribe(c);
    clear b c;
    rethrow(err);
end

unsubscribe(c);
clear b c;
if fid > 0
    fclose(fid);
    disp("Zapisano dane do pliku: " + logFileName);
end
disp("Rozlaczono.");

%% ------------------- FUNKCJA LOKALNA -------------------
function onNotification(src, fig, figCube, b, fid)
    if ~ishandle(fig)
        return;
    end

    % 1) Jesli urzadzenie jest juz rozlaczone, nie przetwarzaj dalej -
    %    to zatrzymuje "dorysowywanie" zaleglych probek z kolejki BLE.
    if ~isvalid(b) || ~b.Connected
        return;
    end

    % W BLE dane pobiera sie funkcja read (nie ma wlasciwosci .Data);
    % timestamp mowi, kiedy probka faktycznie dotarla do komputera.
    [data, timestamp] = read(src, 'oldest');

    % 2) Wiek probki - jesli jest "stara" (kolejka sie spietrzyla),
    %    aktualizujemy tylko bufor danych (szybko), bez kosztownego
    %    rysowania. Dzieki temu MATLAB blyskawicznie "dogania" kolejke
    %    zamiast klatka po klatce odtwarzac zalegle dane.
    wiekProbki = seconds(datetime("now") - timestamp);
    pomijRysowanie = wiekProbki > 0.25;

    raw = typecast(uint8(data), "int16");
    if numel(raw) ~= 6
        return;
    end

    ax = double(raw(1)) / 1000; ay = double(raw(2)) / 1000; az = double(raw(3)) / 1000;
    gx = double(raw(4)) / 10;   gy = double(raw(5)) / 10;   gz = double(raw(6)) / 10;
    vals = [ax; ay; az; gx; gy; gz];

    % Zapis do CSV - kazda probka (surowe wartosci), niezaleznie od tego,
    % czy zostanie narysowana. Format timestamp: milisekundowa precyzja.
    if fid > 0
        fprintf(fid, "%s,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n", ...
            string(timestamp, "yyyy-MM-dd HH:mm:ss.SSS"), ax, ay, az, gx, gy, gz);
    end

    %% Filtr dolnoprzepustowy (EMA) - wygladza szum pomiarowy w czasie rzeczywistym
    % alphaFiltr blizej 1 = mniej wygladzania (szybsza reakcja),
    % blizej 0 = mocniejsze wygladzanie (wolniejsza reakcja).
    alphaFiltr = 0.25;

    ud = fig.UserData;
    if any(isnan(ud.filtPrev))
        filtVals = vals;              % pierwsza probka - brak historii do wygladzania
    else
        filtVals = alphaFiltr * vals + (1 - alphaFiltr) * ud.filtPrev;
    end
    ud.filtPrev = filtVals;

    %% Aktualizacja bufora wykresow czasowych (zawsze - to jest tanie)
    ud.idx = ud.idx + 1;
    if ud.idx > size(ud.buf, 2)
        ud.buf = [ud.buf(:, 2:end), nan(6, 1)];
        ud.idx = size(ud.buf, 2);
    end
    ud.buf(:, ud.idx) = filtVals;
    fig.UserData = ud;

    %% Aktualizacja orientacji szescianu (filtr komplementarny)
    if ishandle(figCube)
        cd = figCube.UserData;

        dt = toc(cd.lastTime);
        cd.lastTime = tic;
        dt = min(dt, 0.2);   % zabezpieczenie przed skokiem po dluzszej przerwie

        accelRoll  = atan2(ay, az);
        accelPitch = atan2(-ax, sqrt(ay^2 + az^2));

        alpha = 0.98;
        cd.roll  = alpha * (cd.roll  + deg2rad(gx) * dt) + (1 - alpha) * accelRoll;
        cd.pitch = alpha * (cd.pitch + deg2rad(gy) * dt) + (1 - alpha) * accelPitch;
        cd.yaw   = cd.yaw + deg2rad(gz) * dt;

        figCube.UserData = cd;
    end

    % 3) Jesli probka jest zalegla (backlog) - pomijamy rysowanie w tej
    %    iteracji, zeby jak najszybciej przetworzyc kolejke do konca.
    if pomijRysowanie
        return;
    end

    % 4) Rysowanie throttlowane (nie na kazdej probce) - mniejsze
    %    obciazenie na callback = mniejsza szansa na spietrzenie kolejki.
    persistent licznik
    if isempty(licznik), licznik = 0; end
    licznik = licznik + 1;
    if mod(licznik, 2) ~= 0   % rysuj co druga probka (~10 FPS przy 20 Hz danych)
        return;
    end

    % Usuniecie trendu (powolnego dryfu/offsetu) na widocznym oknie danych.
    % 'linear' usuwa zarowno stala skladowa jak i powolny sklon (np. dryf
    % zyroskopu); przy krotkim oknie (<6 probek) nie ma sensu detrendowac.
    oknoMin = 6;
    for i = 1:6
        segment = ud.buf(i, 1:ud.idx);
        if ud.idx >= oknoMin
            segment = detrend(segment, "linear");
        end
        set(ud.lines(i), "XData", 1:ud.idx, "YData", segment);
    end

    if ishandle(figCube)
        T = makehgtform("zrotate", cd.yaw, "yrotate", cd.pitch, "xrotate", cd.roll);
        set(cd.hg, "Matrix", T);
    end

    drawnow limitrate;
end
