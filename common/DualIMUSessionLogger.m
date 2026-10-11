classdef DualIMUSessionLogger < handle
    % DUALIMUSESSIONLOGGER Buffered, recoverable two-IMU MAT-file logger.

    properties
        CompletedNormally (1,1) logical = false
    end

    properties (SetAccess = private)
        RawFile (1,1) string
        MetadataFile (1,1) string
        LoggedSamples (1,1) double = 0
        KickCount (1,2) double = [0, 0]
        InvalidFrameCount (1,2) double = [0, 0]
        LoopDurationHistory (:,1) double = zeros(0, 1)
        Closed (1,1) logical = false
    end

    properties (Access = private)
        Metadata (1,1) struct
        BufferCapacity (1,1) double
        BufferCount (1,1) double = 0
        TimeBuffer double
        RawAccelBuffer double
        CalibratedAccelBuffer double
        CorrectedGyroBuffer double
        TemperatureBuffer double
        YprBuffer double
        HitBuffer logical
        FrameValidBuffer logical
        LoopDurationBuffer double
        TargetIndexBuffer uint8
        IMUReadWindowBuffer double
        EMGVoltageBuffer double
        EMGReadWindowBuffer double
        EMGChannelCount (1,1) double = 0
        RawLog = []
    end

    methods
        function obj = DualIMUSessionLogger( ...
                rawFile, metadataFile, metadata, bufferCapacity, emgChannelCount)
            arguments
                rawFile (1,1) string
                metadataFile (1,1) string
                metadata (1,1) struct
                bufferCapacity (1,1) double {mustBeInteger, mustBePositive}
                emgChannelCount (1,1) double {mustBeInteger, mustBeNonnegative} = 0
            end
            obj.RawFile = rawFile;
            obj.MetadataFile = metadataFile;
            obj.Metadata = metadata;
            obj.BufferCapacity = bufferCapacity;
            obj.EMGChannelCount = emgChannelCount;
            obj.TimeBuffer = zeros(bufferCapacity, 1);
            obj.RawAccelBuffer = zeros(bufferCapacity, 3, 2);
            obj.CalibratedAccelBuffer = zeros(bufferCapacity, 3, 2);
            obj.CorrectedGyroBuffer = zeros(bufferCapacity, 3, 2);
            obj.TemperatureBuffer = zeros(bufferCapacity, 2);
            obj.YprBuffer = zeros(bufferCapacity, 3, 2);
            obj.HitBuffer = false(bufferCapacity, 2);
            obj.FrameValidBuffer = true(bufferCapacity, 2);
            obj.LoopDurationBuffer = nan(bufferCapacity, 1);
            obj.TargetIndexBuffer = zeros(bufferCapacity, 2, 'uint8');
            obj.IMUReadWindowBuffer = nan(bufferCapacity, 2);
            obj.EMGVoltageBuffer = nan(bufferCapacity, emgChannelCount);
            obj.EMGReadWindowBuffer = nan(bufferCapacity, 2);
            save(obj.MetadataFile, 'metadata');
        end

        function append(obj, timeSeconds, rawAccelG, calibratedAccelG, ...
                correctedGyroRadPerSec, temperatureC, yprDeg, kickHit, varargin)
            if obj.Closed
                error('MyoDrum:LoggerClosed', ...
                    'Cannot append after the session logger has closed.');
            end
            obj.BufferCount = obj.BufferCount + 1;
            row = obj.BufferCount;
            obj.TimeBuffer(row) = timeSeconds;
            obj.RawAccelBuffer(row, :, :) = rawAccelG;
            obj.CalibratedAccelBuffer(row, :, :) = calibratedAccelG;
            obj.CorrectedGyroBuffer(row, :, :) = correctedGyroRadPerSec;
            obj.TemperatureBuffer(row, :) = temperatureC;
            obj.YprBuffer(row, :, :) = yprDeg;
            obj.HitBuffer(row, :) = kickHit;
            frameValid = true(1, 2);
            loopDurationSeconds = NaN;
            if ~isempty(varargin)
                frameValid = varargin{1};
            end
            if numel(varargin) >= 2
                loopDurationSeconds = varargin{2};
            end
            targetIndex = zeros(1, 2, 'uint8');
            if numel(varargin) >= 3
                targetIndex = uint8(varargin{3});
            end
            imuReadWindow = [NaN, NaN];
            emgVoltages = nan(1, obj.EMGChannelCount);
            emgReadWindow = [NaN, NaN];
            if numel(varargin) >= 4
                imuReadWindow = varargin{4};
            end
            if numel(varargin) >= 5
                emgVoltages = varargin{5};
            end
            if numel(varargin) >= 6
                emgReadWindow = varargin{6};
            end
            if ~isequal(size(frameValid), [1, 2])
                error('MyoDrum:InvalidFrameValidity', ...
                    'frameValid must be a 1-by-2 logical vector.');
            end
            obj.FrameValidBuffer(row, :) = logical(frameValid);
            obj.LoopDurationBuffer(row) = loopDurationSeconds;
            obj.TargetIndexBuffer(row, :) = targetIndex;
            obj.IMUReadWindowBuffer(row, :) = imuReadWindow;
            if obj.EMGChannelCount > 0
                if ~isequal(size(emgVoltages), [1, obj.EMGChannelCount])
                    error('MyoDrum:InvalidEMGSet', ...
                        'EMG voltages must be a 1-by-%d vector.', ...
                        obj.EMGChannelCount);
                end
                obj.EMGVoltageBuffer(row, :) = emgVoltages;
                obj.EMGReadWindowBuffer(row, :) = emgReadWindow;
            end
            obj.KickCount = obj.KickCount + double(kickHit);
            obj.InvalidFrameCount = obj.InvalidFrameCount + ...
                double(~frameValid);
            obj.LoopDurationHistory(end + 1, 1) = loopDurationSeconds;
            if obj.BufferCount == obj.BufferCapacity
                obj.flush();
            end
        end

        function flush(obj)
            if obj.BufferCount == 0
                return;
            end
            sourceRows = 1:obj.BufferCount;
            destinationRows = obj.LoggedSamples + (1:obj.BufferCount);
            if isempty(obj.RawLog)
                initialSessionId = obj.Metadata.sessionId;
                save(obj.RawFile, 'initialSessionId', '-v7.3');
                obj.RawLog = matfile(obj.RawFile, 'Writable', true);
            end
            obj.RawLog.time_s(destinationRows, 1) = ...
                obj.TimeBuffer(sourceRows, 1);
            obj.RawLog.accelRaw_g(destinationRows, 1:3, 1:2) = ...
                obj.RawAccelBuffer(sourceRows, :, :);
            obj.RawLog.accelCalibrated_g(destinationRows, 1:3, 1:2) = ...
                obj.CalibratedAccelBuffer(sourceRows, :, :);
            obj.RawLog.gyroCorrected_rad_s(destinationRows, 1:3, 1:2) = ...
                obj.CorrectedGyroBuffer(sourceRows, :, :);
            obj.RawLog.temperature_C(destinationRows, 1:2) = ...
                obj.TemperatureBuffer(sourceRows, :);
            obj.RawLog.yawPitchRoll_deg(destinationRows, 1:3, 1:2) = ...
                obj.YprBuffer(sourceRows, :, :);
            obj.RawLog.kickHit(destinationRows, 1:2) = ...
                obj.HitBuffer(sourceRows, :);
            obj.RawLog.frameValid(destinationRows, 1:2) = ...
                obj.FrameValidBuffer(sourceRows, :);
            obj.RawLog.loopDuration_s(destinationRows, 1) = ...
                obj.LoopDurationBuffer(sourceRows, 1);
            obj.RawLog.targetIndex(destinationRows, 1:2) = ...
                obj.TargetIndexBuffer(sourceRows, :);
            obj.RawLog.imuReadStart_s(destinationRows, 1) = ...
                obj.IMUReadWindowBuffer(sourceRows, 1);
            obj.RawLog.imuReadEnd_s(destinationRows, 1) = ...
                obj.IMUReadWindowBuffer(sourceRows, 2);
            obj.RawLog.imuTimestamp_s(destinationRows, 1) = mean( ...
                obj.IMUReadWindowBuffer(sourceRows, :), 2);
            if obj.EMGChannelCount > 0
                obj.RawLog.emgVoltage_V(destinationRows, ...
                    1:obj.EMGChannelCount) = ...
                    obj.EMGVoltageBuffer(sourceRows, :);
                obj.RawLog.emgReadStart_s(destinationRows, 1) = ...
                    obj.EMGReadWindowBuffer(sourceRows, 1);
                obj.RawLog.emgReadEnd_s(destinationRows, 1) = ...
                    obj.EMGReadWindowBuffer(sourceRows, 2);
                obj.RawLog.emgTimestamp_s(destinationRows, 1) = mean( ...
                    obj.EMGReadWindowBuffer(sourceRows, :), 2);
            end
            obj.LoggedSamples = obj.LoggedSamples + obj.BufferCount;
            obj.BufferCount = 0;
        end

        function close(obj)
            if obj.Closed
                return;
            end
            obj.flush();
            metadata = obj.Metadata;
            metadata.endedAt = string(datetime('now', 'TimeZone', 'local'));
            metadata.loggedSamples = obj.LoggedSamples;
            metadata.kickCount = obj.KickCount;
            metadata.invalidFrameCount = obj.InvalidFrameCount;
            metadata.completedNormally = obj.CompletedNormally;
            save(obj.MetadataFile, 'metadata');
            obj.Closed = true;
        end
    end
end
