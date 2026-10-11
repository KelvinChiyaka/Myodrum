function [frames, remaining, diagnostics, lastSequence] = ...
        decodeIMUEMGStreamBytes(bytes, lastSequence)
% DECODEIMUEMGSTREAMBYTES Decode synchronized two-IMU/four-EMG packets.

    arguments
        bytes (:,1) uint8
        lastSequence = []
    end
    frameLength = 47;
    sync = uint8([165; 90]);
    bytes = bytes(:);
    frames.sequence = zeros(0, 1, 'uint16');
    frames.timestamp_us = zeros(0, 1, 'uint32');
    frames.accelG = zeros(0, 3, 2);
    frames.gyroDps = zeros(0, 3, 2);
    frames.temperatureC = zeros(0, 2);
    frames.emgAdc = zeros(0, 4, 'uint16');
    frames.status = zeros(0, 1, 'uint8');
    frames.frameValid = false(0, 2);
    diagnostics.checksumFailures = 0;
    diagnostics.parseResets = 0;
    diagnostics.droppedFrames = 0;
    cursor = 1;
    while numel(bytes) - cursor + 1 >= 2
        relative = find(bytes(cursor:end-1) == sync(1) & ...
            bytes(cursor+1:end) == sync(2), 1);
        if isempty(relative)
            remaining = zeros(0, 1, 'uint8');
            if bytes(end) == sync(1), remaining = bytes(end); end
            if numel(bytes) >= cursor
                diagnostics.parseResets = diagnostics.parseResets + 1;
            end
            return;
        end
        first = cursor + relative - 1;
        if first > cursor
            diagnostics.parseResets = diagnostics.parseResets + 1;
        end
        if numel(bytes) - first + 1 < frameLength
            remaining = bytes(first:end);
            return;
        end
        packet = bytes(first:first + frameLength - 1);
        if packet(3) ~= 1 || crc8EMG(packet(3:46)) ~= packet(47)
            diagnostics.checksumFailures = diagnostics.checksumFailures + 1;
            cursor = first + 1;
            continue;
        end
        sequence = readLE(packet(4:5));
        timestamp = readLE(packet(6:9));
        status = packet(46);
        row = numel(frames.sequence) + 1;
        frames.sequence(row, 1) = uint16(sequence);
        frames.timestamp_us(row, 1) = uint32(timestamp);
        frames.status(row, 1) = status;
        for sensor = 1:2
            offset = 10 + (sensor - 1) * 14;
            raw = packet(offset:offset + 13);
            values = zeros(1, 7);
            for valueIndex = 1:7
                at = 1 + 2 * (valueIndex - 1);
                values(valueIndex) = signedBE(raw(at:at + 1));
            end
            frames.accelG(row, :, sensor) = values(1:3) / 16384;
            frames.temperatureC(row, sensor) = values(4) / 340 + 36.53;
            frames.gyroDps(row, :, sensor) = values(5:7) / 32.8;
            frames.frameValid(row, sensor) = bitand(status, ...
                bitshift(uint8(1), sensor - 1)) == 0;
        end
        for channel = 1:4
            offset = 38 + 2 * (channel - 1);
            frames.emgAdc(row, channel) = uint16(readLE( ...
                packet(offset:offset + 1)));
        end
        if ~isempty(lastSequence)
            gap = mod(double(sequence) - double(lastSequence), 65536) - 1;
            if gap > 0
                diagnostics.droppedFrames = diagnostics.droppedFrames + gap;
            end
        end
        lastSequence = uint16(sequence);
        cursor = first + frameLength;
    end
    remaining = bytes(cursor:end);
end

function value = readLE(bytes)
    value = uint32(0);
    for index = 1:numel(bytes)
        value = value + bitshift(uint32(bytes(index)), 8 * (index - 1));
    end
end

function value = signedBE(bytes)
    unsigned = uint16(bytes(1)) * 256 + uint16(bytes(2));
    value = double(typecast(unsigned, 'int16'));
end
