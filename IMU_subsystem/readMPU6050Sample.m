function sample = readMPU6050Sample(sensor)
% READMPU6050SAMPLE Robustly read one accel/gyro frame from an MPU6050.

    maxAttempts = 20;
    raw = uint8([]);
    for attempt = 1:maxAttempts
        raw = readRegister(sensor.dev, hex2dec('3B'), 14, 'uint8');
        raw = raw(:).';
        if numel(raw) == 14 && any(raw ~= 0)
            break;
        end

        if attempt == 5 || attempt == 12
            try
                reapplyConfiguration(sensor);
            catch
                % Continue retrying; the final error contains the address.
            end
        end
        pause(0.02);
    end

    if numel(raw) ~= 14 || all(raw == 0)
        error('MyoDrum:InvalidIMUFrame', ...
            ['MPU6050 at %s returned no valid frame after %d attempts. ' ...
             'Check power, ground, SDA, SCL, and AD0.'], ...
            sensor.address, maxAttempts);
    end

    sample.accelG = [ ...
        signed16(raw(1), raw(2)), ...
        signed16(raw(3), raw(4)), ...
        signed16(raw(5), raw(6))] / 16384;
    sample.temperatureC = signed16(raw(7), raw(8)) / 340 + 36.53;
    sample.gyroDps = [ ...
        signed16(raw(9), raw(10)), ...
        signed16(raw(11), raw(12)), ...
        signed16(raw(13), raw(14))] / 131;
end

function reapplyConfiguration(sensor)
    for registerIndex = 1:size(sensor.initRegs, 1)
        writeRegister(sensor.dev, sensor.initRegs(registerIndex, 1), ...
            uint8(sensor.initRegs(registerIndex, 2)), 'uint8');
        if registerIndex == 1
            pause(0.10);
        end
    end
end

function value = signed16(highByte, lowByte)
    bits = bitor(bitshift(uint16(highByte), 8), uint16(lowByte));
    value = double(bits);
    if bits >= 32768
        value = value - 65536;
    end
end
