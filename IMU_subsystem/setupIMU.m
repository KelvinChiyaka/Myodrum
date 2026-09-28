function [imu1, imu2, imu1Connected, imu2Connected] = setupIMU(a)

    %% CHECK FOR IMUs
    disp('Checking for IMUs...');
    disp(' ');

    imu1 = [];
    imu2 = [];
    imu1Connected = false;
    imu2Connected = false;

    % IMU 1 - Address 0x68
    try
        imu1 = device(a, 'I2CAddress', '0x68');
        id1 = readRegister(imu1, hex2dec('75'), 1, 'uint8');

        if id1 == hex2dec('68')
            imu1Connected = true;
            disp('IMU 1 connected  (0x68)');
        else
            % FIX 2: something answered, but it is not an MPU6050
            fprintf('IMU 1 found at 0x68 but WHO_AM_I = 0x%02X (expected 0x68)\n', id1);
        end
    catch err
        fprintf('IMU 1 not detected (%s)\n', err.message);
    end

    % IMU 2 - Address 0x69
    try
        imu2 = device(a, 'I2CAddress', '0x69');
        id2 = readRegister(imu2, hex2dec('75'), 1, 'uint8');

        if id2 == hex2dec('68')
            imu2Connected = true;
            disp('IMU 2 connected  (0x69)');
        else
            fprintf('IMU 2 found at 0x69 but WHO_AM_I = 0x%02X (expected 0x68)\n', id2);
        end
    catch err
        fprintf('IMU 2 not detected (%s)\n', err.message);
    end

    %% CHECK THAT AT LEAST ONE IMU EXISTS
    if ~imu1Connected && ~imu2Connected
        error('No MPU6050 IMUs detected. Check your wiring.');
    end
    disp(' ');

    %% INITIALISE CONNECTED IMUs
    PWR_MGMT_1   = hex2dec('6B');
    ACCEL_CONFIG = hex2dec('1C');
    GYRO_CONFIG  = hex2dec('1B');

    if imu1Connected
        writeRegister(imu1, PWR_MGMT_1, 0, 'uint8');
        writeRegister(imu1, ACCEL_CONFIG, 0, 'uint8');
        writeRegister(imu1, GYRO_CONFIG, 0, 'uint8');
    end

    if imu2Connected
        writeRegister(imu2, PWR_MGMT_1, 0, 'uint8');
        writeRegister(imu2, ACCEL_CONFIG, 0, 'uint8');
        writeRegister(imu2, GYRO_CONFIG, 0, 'uint8');
    end
end