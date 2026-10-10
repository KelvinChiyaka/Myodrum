classdef FakeEMGStreamReader < handle
    % FAKEEMGSTREAMREADER Test-only source of decoded firmware frames.

    properties
        Sequence (1,1) uint16 = uint16(0)
        Timestamp (1,1) uint32 = uint32(0)
        Running (1,1) logical = false
        FramesReceived (1,1) double = 0
    end

    methods
        function start(obj)
            obj.Running = true;
        end

        function stop(obj)
            obj.Running = false;
        end

        function frames = readAvailableFrames(obj)
            count = 4 * double(obj.Running);
            frames.sequence = zeros(count, 1, 'uint16');
            frames.timestamp_us = zeros(count, 1, 'uint32');
            frames.adc = zeros(count, 4, 'uint16');
            frames.status = zeros(count, 1, 'uint8');
            frames.time_s = zeros(count, 1);
            for row = 1:count
                frames.sequence(row) = obj.Sequence;
                frames.timestamp_us(row) = obj.Timestamp;
                frames.time_s(row) = double(obj.Timestamp) / 1e6;
                frames.adc(row, :) = uint16([0, 200, 400, 600] + ...
                    mod(double(obj.Sequence), 20));
                obj.Sequence = obj.Sequence + 1;
                obj.Timestamp = obj.Timestamp + uint32(4000);
            end
            obj.FramesReceived = obj.FramesReceived + count;
        end

        function stats = getStatistics(obj)
            stats.framesReceived = obj.FramesReceived;
            stats.droppedFrames = 0;
            stats.checksumFailures = 0;
            stats.parseResets = 0;
            stats.measuredRateHz = 250;
        end
    end
end
