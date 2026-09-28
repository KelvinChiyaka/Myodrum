clear;
clc;

%% SETTINGS
COM_PORT = 'COM8';

%% CONNECT TO ARDUINO
disp('MyoDrum IMU System');
disp('Connecting to Arduino Mega 2560...');
a = arduino(COM_PORT, 'Mega2560', 'Libraries', 'I2C');
disp('Arduino connected.');
disp(' ');

%% DETECT AND INITIALISE IMUs
[imu1, imu2, imu1Connected, imu2Connected] = setupIMUs(a);

%% STATIC CALIBRATION (runs once, follow the prompts)
static = staticCalibration(imu1, imu2, imu1Connected, imu2Connected);

%% START MEASUREMENTS
disp(' ');
disp('Starting measurements... Press Ctrl+C to stop.');
disp(' ');

while true
    data = readIMUI2C(imu1, imu2, imu1Connected, imu2Connected);
    imus = {data.IMU1, data.IMU2};

    for k = 1:2
        if imus{k}.connected
            % Apply calibration (row k of each matrix belongs to IMU k)
            accel = (imus{k}.accel - static.accelOffset(k, :)) ./ static.accelScale(k, :);
            gyro  =  imus{k}.gyro  - static.gyroBias(k, :);

            fprintf(['IMU %d | Acc: X=%6.2f Y=%6.2f Z=%6.2f g | ' ...
                     'Gyro: X=%7.2f Y=%7.2f Z=%7.2f deg/s\n'], ...
                     k, accel, gyro);
        end
    end

    pause(0.01);
end