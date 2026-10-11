classdef IMUEMGStreamReader < handle
    % IMUEMGSTREAMREADER Non-blocking synchronized sensor stream reader.
    properties (SetAccess = private)
        FramesReceived (1,1) double = 0
        DroppedFrames (1,1) double = 0
        ChecksumFailures (1,1) double = 0
        ParseResets (1,1) double = 0
    end
    properties (Access = private)
        Port
        ByteBuffer (:,1) uint8 = zeros(0, 1, 'uint8')
        LastSequence = []
        PreviousTimestamp = []
        TimestampWraps (1,1) double = 0
        FirstExtendedTimestamp = []
        LastExtendedTimestamp = []
    end
    methods
        function obj = IMUEMGStreamReader(portName, baudRate)
            arguments
                portName (1,1) string
                baudRate (1,1) double {mustBeInteger, mustBePositive}
            end
            available = string(serialportlist("available"));
            if ~any(strcmpi(portName, available))
                error('MyoDrum:CombinedStreamPortUnavailable', ...
                    'Port %s is unavailable. Available: %s.', ...
                    portName, strjoin(available, ', '));
            end
            obj.Port = serialport(portName, baudRate, 'Timeout', 1);
            pause(2);
            flush(obj.Port);
        end
        function start(obj)
            flush(obj.Port);
            write(obj.Port, uint8('S'), 'uint8');
        end
        function stop(obj)
            if ~isempty(obj.Port), write(obj.Port, uint8('T'), 'uint8'); end
        end
        function frames = readAvailableFrames(obj)
            available = obj.Port.NumBytesAvailable;
            if available > 0
                incoming = read(obj.Port, available, 'uint8');
                obj.ByteBuffer = [obj.ByteBuffer; incoming(:)];
            end
            [frames, obj.ByteBuffer, diagnostic, obj.LastSequence] = ...
                decodeIMUEMGStreamBytes(obj.ByteBuffer, obj.LastSequence);
            obj.FramesReceived = obj.FramesReceived + numel(frames.sequence);
            obj.DroppedFrames = obj.DroppedFrames + diagnostic.droppedFrames;
            obj.ChecksumFailures = obj.ChecksumFailures + ...
                diagnostic.checksumFailures;
            obj.ParseResets = obj.ParseResets + diagnostic.parseResets;
            frames.time_s = obj.extendTimestamps(frames.timestamp_us);
        end
        function statistics = getStatistics(obj)
            statistics.framesReceived = obj.FramesReceived;
            statistics.droppedFrames = obj.DroppedFrames;
            statistics.checksumFailures = obj.ChecksumFailures;
            statistics.parseResets = obj.ParseResets;
            if obj.FramesReceived > 1 && ...
                    obj.LastExtendedTimestamp > obj.FirstExtendedTimestamp
                elapsed = (obj.LastExtendedTimestamp - ...
                    obj.FirstExtendedTimestamp) / 1e6;
                statistics.measuredRateHz = (obj.FramesReceived - 1) / elapsed;
            else
                statistics.measuredRateHz = NaN;
            end
        end
        function delete(obj)
            try, obj.stop(); catch, end
            obj.Port = [];
        end
    end
    methods (Access = private)
        function seconds = extendTimestamps(obj, timestamps)
            seconds = zeros(numel(timestamps), 1);
            for index = 1:numel(timestamps)
                current = timestamps(index);
                if ~isempty(obj.PreviousTimestamp) && ...
                        current < obj.PreviousTimestamp && ...
                        double(obj.PreviousTimestamp)-double(current) > 2^31
                    obj.TimestampWraps = obj.TimestampWraps + 1;
                end
                extended = double(current) + obj.TimestampWraps * 2^32;
                if isempty(obj.FirstExtendedTimestamp)
                    obj.FirstExtendedTimestamp = extended;
                end
                obj.LastExtendedTimestamp = extended;
                obj.PreviousTimestamp = current;
                seconds(index) = (extended - obj.FirstExtendedTimestamp) / 1e6;
            end
        end
    end
end
