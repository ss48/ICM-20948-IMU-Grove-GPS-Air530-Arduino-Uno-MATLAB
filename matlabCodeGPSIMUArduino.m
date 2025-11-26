port = "COM4";   % change to your Uno port
baud = 115200;

s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);

% Sample rate ~ 20 Hz (delay(50) in Arduino)
Fs = 20;
compFilt = complementaryFilter( ...
    "SampleRate", Fs, ...
    "HasMagnetometer", true);
tuner = HelperOrientationFilterTuner(compFilt);

runTime = 60;   % seconds
tic
while toc < runTime
    try
        line = readline(s);
    catch
        continue;
    end

    line = strtrim(line);
    if line == ""
        continue;
    end

    parts = split(line, ",");
    if numel(parts) ~= 19
        % roll,pitch,yaw, accX,Y,Z, gyroX,Y,Z, magX,Y,Z,
        % press,alt,temp, lat,lon,alt_gps  -> 19 fields
        continue;
    end

    vals = str2double(parts);
    if any(isnan(vals(1:15)))
        % At least IMU/pressure/temp must be valid
        continue;
    end

    roll  = vals(1);
    pitch = vals(2);
    yaw   = vals(3);

    acc   = vals(4:6).';
    gyro  = vals(7:9).';
    mag   = vals(10:12).';

    press_hPa = vals(13);
    alt_m     = vals(14);
    temp_C    = vals(15);

    lat       = vals(16);
    lon       = vals(17);
    alt_gps   = vals(18);

    % If you want to re-run your own complementaryFilter on raw data:
    q = compFilt(acc.', gyro.', mag.');
    update(tuner, q);

    % Simple console feedback
    fprintf("Roll: %6.2f Pitch: %6.2f Yaw: %6.2f | Lat: %.6f Lon: %.6f\n", ...
            roll, pitch, yaw, lat, lon);
end

clear s
