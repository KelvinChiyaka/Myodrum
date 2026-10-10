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
        RawLog = []
    end

    methods
        function obj = DualIMUSessionLogger( ...
                rawFile, metadataFile, metadata, bufferCapacity)
            arguments
                rawFile (1,1) string
                metadataFile (1,1) string
                metadata (1,1) struct
                bufferCapacity (1,1) double {mustBeInteger, mustBePositive}
            end
            obj.RawFile = rawFile;
            obj.MetadataFile = metadataFile;
            obj.Metadata = metadata;
            obj.BufferCapacity = bufferCapacity;
            obj.TimeBuffer = zeros(bufferCapacity, 1);
            obj.RawAccelBuffer = zeros(bufferCapacity, 3, 2);
            obj.CalibratedAccelBuffer = zeros(bufferCapacity, 3, 2);
            obj.CorrectedGyroBuffer = zeros(bufferCapacity, 3, 2);
            obj.TemperatureBuffer = zeros(bufferCapacity, 2);
            obj.YprBuffer = zeros(bufferCapacity, 3, 2);
            obj.HitBuffer = false(bufferCapacity, 2);
            save(obj.MetadataFile, 'metadata');
        end

        function append(obj, timeSeconds, rawAccelG, calibratedAccelG, ...
                correctedGyroRadPerSec, temperatureC, yprDeg, kickHit)
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
            obj.KickCount = obj.KickCount + double(kickHit);
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
            metadata.completedNormally = obj.CompletedNormally;
            save(obj.MetadataFile, 'metadata');
            obj.Closed = true;
        end
    end
end
