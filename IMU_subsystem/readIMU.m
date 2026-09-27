%% MyoDrum - MPU6050 I2C Reader

% IMU 1 -> 0x68
% IMU 2 -> 0x69

% Mega 2560:
% SDA -> Pin 20
% SCL -> Pin 21

clear;
clc;


%% 1. CONNECT TO ARDUINO

COM_PORT = 'COM8';

disp('Connecting to Arduino Mega 2560...');

a = arduino(COM_PORT, 'Mega2560', 'Libraries', 'I2C');

disp('Arduino connected.');
disp(' ');



%% 2. CHECK FOR IMUs


disp('Checking for IMUs...');

% Check IMU 1
try
    imu1 = device(a, 'I2CAddress', '0x68');

    % Try to read WHO_AM_I
    id1 = readRegister(imu1, hex2dec('75'), 1, 'uint8');

    if id1 == hex2dec('68')
        imu1Connected = true;
        disp('IMU 1 connected  (0x68)');
    else
        imu1Connected = false;
        disp('IMU 1 not detected');
    end

catch
    imu1Connected = false;
    disp('IMU 1 not detected');
end


% Check IMU 2
try
    imu2 = device(a, 'I2CAddress', '0x69');

    % Try to read WHO_AM_I
    id2 = readRegister(imu2, hex2dec('75'), 1, 'uint8');

    if id2 == hex2dec('68')
        imu2Connected = true;
        disp('IMU 2 connected  (0x69)');
    else
        imu2Connected = false;
        disp('IMU 2 not detected');
    end

catch
    imu2Connected = false;
    disp('IMU 2 not detected');
end


%% 3. STOP IF NO IMUs ARE CONNECTED

if ~imu1Connected && ~imu2Connected

    error('No MPU6050 IMUs detected. Check your wiring.');

end


%% 4. INITIALISE CONNECTED IMUs

PWR_MGMT_1   = hex2dec('6B');
ACCEL_CONFIG = hex2dec('1C');
GYRO_CONFIG  = hex2dec('1B');

% IMU 1
if imu1Connected

    writeRegister(imu1, PWR_MGMT_1, 0, 'uint8');
    writeRegister(imu1, ACCEL_CONFIG, 0, 'uint8');
    writeRegister(imu1, GYRO_CONFIG, 0, 'uint8');

end

% IMU 2
if imu2Connected

    writeRegister(imu2, PWR_MGMT_1, 0, 'uint8');
    writeRegister(imu2, ACCEL_CONFIG, 0, 'uint8');
    writeRegister(imu2, GYRO_CONFIG, 0, 'uint8');

end


%% 5. START READING

DATA_START = hex2dec('3B');

disp(' ');
disp('==========================================');
disp('Starting IMU measurements...');
disp('Press Ctrl+C to stop.');
disp('==========================================');
disp(' ');


while true

    %% IMU 1

    if imu1Connected

        data1 = readRegister(imu1, DATA_START, 14, 'uint8');

        % Accelerometer
        ax1 = combineBytes(data1(1), data1(2)) / 16384;
        ay1 = combineBytes(data1(3), data1(4)) / 16384;
        az1 = combineBytes(data1(5), data1(6)) / 16384;

        % Gyroscope
        gx1 = combineBytes(data1(9),  data1(10)) / 131;
        gy1 = combineBytes(data1(11), data1(12)) / 131;
        gz1 = combineBytes(data1(13), data1(14)) / 131;

        fprintf(['IMU 1 | Acc: X=%6.2f Y=%6.2f Z=%6.2f g | ' ...
                 'Gyro: X=%7.2f Y=%7.2f Z=%7.2f deg/s\n'], ...
                 ax1, ay1, az1, gx1, gy1, gz1);

    end

    %% IMU 2

    if imu2Connected

        data2 = readRegister(imu2, DATA_START, 14, 'uint8');

        % Accelerometer
        ax2 = combineBytes(data2(1), data2(2)) / 16384;
        ay2 = combineBytes(data2(3), data2(4)) / 16384;
        az2 = combineBytes(data2(5), data2(6)) / 16384;

        % Gyroscope
        gx2 = combineBytes(data2(9),  data2(10)) / 131;
        gy2 = combineBytes(data2(11), data2(12)) / 131;
        gz2 = combineBytes(data2(13), data2(14)) / 131;

        fprintf(['IMU 2 | Acc: X=%6.2f Y=%6.2f Z=%6.2f g | ' ...
                 'Gyro: X=%7.2f Y=%7.2f Z=%7.2f deg/s\n'], ...
                 ax2, ay2, az2, gx2, gy2, gz2);

    end


    disp(' ');

    pause(0.01);

end

%% FUNCTION: COMBINE TWO BYTES

function value = combineBytes(highByte, lowByte)

    value = bitor(bitshift(uint16(highByte), 8), ...
                  uint16(lowByte));

    if value >= 32768
        value = double(value) - 65536;
    else
        value = double(value);
    end

end