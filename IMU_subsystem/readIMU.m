function data = readIMU(imu1, imu2, imu1Connected, imu2Connected)
% READIMUI2C Read one sample from the connected MPU6050 IMUs.
%
% IMU 1 -> 0x68
% IMU 2 -> 0x69
%
% Accelerometer output: g
% Gyroscope output: deg/s

    DATA_START = hex2dec('3B');

    % Create empty output
    data.IMU1.connected = imu1Connected;
    data.IMU2.connected = imu2Connected;
    data.IMU1.accel = [];
    data.IMU1.gyro  = [];
    data.IMU2.accel = [];
    data.IMU2.gyro  = [];

    %% READ IMU 1
    if imu1Connected
        raw = readRegister(imu1, DATA_START, 14, 'uint8');

        data.IMU1.accel = [ ...
            combineBytes(raw(1), raw(2)) / 16384, ...
            combineBytes(raw(3), raw(4)) / 16384, ...
            combineBytes(raw(5), raw(6)) / 16384];

        data.IMU1.gyro = [ ...
            combineBytes(raw(9),  raw(10)) / 131, ...
            combineBytes(raw(11), raw(12)) / 131, ...
            combineBytes(raw(13), raw(14)) / 131];
    end

    %% READ IMU 2
    if imu2Connected
        raw = readRegister(imu2, DATA_START, 14, 'uint8');

        data.IMU2.accel = [ ...
            combineBytes(raw(1), raw(2)) / 16384, ...
            combineBytes(raw(3), raw(4)) / 16384, ...
            combineBytes(raw(5), raw(6)) / 16384];

        data.IMU2.gyro = [ ...
            combineBytes(raw(9),  raw(10)) / 131, ...
            combineBytes(raw(11), raw(12)) / 131, ...
            combineBytes(raw(13), raw(14)) / 131];
    end
end

%% CONVERT TWO BYTES TO SIGNED VALUE
function value = combineBytes(highByte, lowByte)
    value = bitor(bitshift(uint16(highByte), 8), uint16(lowByte));

    if value >= 32768
        value = double(value) - 65536;
    else
        value = double(value);
    end
end