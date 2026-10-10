function sensors = setupMPU6050Pair(board, addresses, outputRateHz, dlpfConfig)
% SETUPMPU6050PAIR Connect and configure two MPU6050 devices.
%
% addresses is normally ["0x68", "0x69"]. Both devices share SDA/SCL;
% the second sensor must have AD0 tied high to select address 0x69.

    arguments
        board (1,1)
        addresses (1,:) string
        outputRateHz (1,1) double {mustBeFinite, mustBePositive}
        dlpfConfig (1,1) double {mustBeInteger, mustBeInRange(dlpfConfig, 0, 6)} = 2
    end

    if numel(addresses) ~= 2 || numel(unique(addresses)) ~= 2
        error('MyoDrum:InvalidIMUAddresses', ...
            'Exactly two different MPU6050 addresses are required.');
    end

    divider = 1000 / outputRateHz - 1;
    if divider < 0 || divider > 255 || divider ~= round(divider)
        error('MyoDrum:InvalidIMURate', ...
            'Sensor output rate must produce an integer divider from 1 kHz.');
    end

    template = struct('dev', [], 'address', "", 'initRegs', zeros(0, 2));
    sensors = repmat(template, 1, 2);
    for sensorIndex = 1:2
        address = addresses(sensorIndex);
        dev = device(board, 'I2CAddress', char(address));
        identity = readRegister(dev, hex2dec('75'), 1, 'uint8');
        if isempty(identity) || identity(1) ~= hex2dec('68')
            error('MyoDrum:WrongIMUIdentity', ...
                'No MPU6050 responded correctly at %s.', address);
        end

        sensors(sensorIndex).dev = dev;
        sensors(sensorIndex).address = address;
        sensors(sensorIndex).initRegs = [ ...
            hex2dec('6B'), 1; ... % clock source; wake device
            hex2dec('1A'), dlpfConfig; ... % DLPF configuration
            hex2dec('19'), divider; ...
            hex2dec('1B'), 0; ... % gyro +/-250 degrees/s
            hex2dec('1C'), 0];    % accelerometer +/-2 g
        configureSensor(sensors(sensorIndex));
        fprintf('MPU6050 %d connected at %s.\n', sensorIndex, address);
    end
end

function configureSensor(sensor)
    for registerIndex = 1:size(sensor.initRegs, 1)
        writeRegister(sensor.dev, sensor.initRegs(registerIndex, 1), ...
            uint8(sensor.initRegs(registerIndex, 2)), 'uint8');
        if registerIndex == 1
            pause(0.10);
        end
    end
    pause(0.05);
end
