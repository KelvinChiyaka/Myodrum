function [frames, remaining, diagnostics, lastSequence] = ...
        decodeEMGStreamBytes(bytes, lastSequence)
% DECODEEMGSTREAMBYTES Decode and resynchronise fixed-length EMG packets.

    arguments
        bytes (:,1) uint8
        lastSequence = []
    end
    frameLength = 19;
    sync = uint8([170; 85]);
    bytes = bytes(:);
    frames.sequence = zeros(0, 1, 'uint16');
    frames.timestamp_us = zeros(0, 1, 'uint32');
    frames.adc = zeros(0, 4, 'uint16');
    frames.status = zeros(0, 1, 'uint8');
    diagnostics.checksumFailures = 0;
    diagnostics.parseResets = 0;
    diagnostics.droppedFrames = 0;

    cursor = 1;
    while numel(bytes) - cursor + 1 >= 2
        relativeSync = find(bytes(cursor:end-1) == sync(1) & ...
            bytes(cursor+1:end) == sync(2), 1);
        if isempty(relativeSync)
            if bytes(end) == sync(1)
                remaining = bytes(end);
            else
                remaining = zeros(0, 1, 'uint8');
            end
            if numel(bytes) >= cursor
                diagnostics.parseResets = diagnostics.parseResets + 1;
            end
            return;
        end
        frameStart = cursor + relativeSync - 1;
        if frameStart > cursor
            diagnostics.parseResets = diagnostics.parseResets + 1;
        end
        if numel(bytes) - frameStart + 1 < frameLength
            remaining = bytes(frameStart:end);
            return;
        end
        packet = bytes(frameStart:frameStart + frameLength - 1);
        if packet(3) ~= 1 || crc8EMG(packet(3:18)) ~= packet(19)
            diagnostics.checksumFailures = diagnostics.checksumFailures + 1;
            cursor = frameStart + 1;
            continue;
        end

        sequence = readUint16(packet(4:5));
        timestamp = readUint32(packet(6:9));
        adcValues = zeros(1, 4, 'uint16');
        for channelIndex = 1:4
            firstByte = 10 + 2 * (channelIndex - 1);
            adcValues(channelIndex) = ...
                readUint16(packet(firstByte:firstByte + 1));
        end
        if ~isempty(lastSequence)
            gap = mod(double(sequence) - double(lastSequence), 65536) - 1;
            if gap > 0
                diagnostics.droppedFrames = ...
                    diagnostics.droppedFrames + gap;
            end
        end
        lastSequence = sequence;
        outputRow = numel(frames.sequence) + 1;
        frames.sequence(outputRow, 1) = sequence;
        frames.timestamp_us(outputRow, 1) = timestamp;
        frames.adc(outputRow, :) = adcValues;
        frames.status(outputRow, 1) = packet(18);
        cursor = frameStart + frameLength;
    end
    remaining = bytes(cursor:end);
end

function value = readUint16(bytes)
    value = uint16(bytes(1)) + bitshift(uint16(bytes(2)), 8);
end

function value = readUint32(bytes)
    value = uint32(bytes(1)) + bitshift(uint32(bytes(2)), 8) + ...
        bitshift(uint32(bytes(3)), 16) + bitshift(uint32(bytes(4)), 24);
end
