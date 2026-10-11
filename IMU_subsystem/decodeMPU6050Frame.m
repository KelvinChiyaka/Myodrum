function sample = decodeMPU6050Frame(raw, gyroLsbPerDps)
% DECODEMPU6050FRAME Decode one 14-byte MPU6050 register frame.
%
% gyroLsbPerDps is 131 for +/-250 dps (default), 65.5 for +/-500,
% 32.8 for +/-1000 and 16.4 for +/-2000.

    arguments
        raw (1,14) {mustBeNumeric}
        gyroLsbPerDps (1,1) double {mustBePositive, mustBeFinite} = 131
    end
    if any(~isfinite(double(raw))) || any(raw < 0) || any(raw > 255) || ...
            any(raw ~= floor(raw))
        error('MyoDrum:InvalidIMUFrameBytes', ...
            'An MPU6050 frame must contain 14 integer bytes from 0 to 255.');
    end
    raw = uint8(raw);
    sample.accelG = [signed16(raw(1), raw(2)), ...
        signed16(raw(3), raw(4)), signed16(raw(5), raw(6))] / 16384;
    sample.temperatureC = signed16(raw(7), raw(8)) / 340 + 36.53;
    sample.gyroDps = [signed16(raw(9), raw(10)), ...
        signed16(raw(11), raw(12)), signed16(raw(13), raw(14))] / ...
        gyroLsbPerDps;
end

function value = signed16(highByte, lowByte)
    bits = bitor(bitshift(uint16(highByte), 8), uint16(lowByte));
    value = double(bits);
    if bits >= 32768
        value = value - 65536;
    end
end
