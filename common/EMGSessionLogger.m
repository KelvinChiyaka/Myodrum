classdef EMGSessionLogger < handle
    % EMGSESSIONLOGGER Buffered, recoverable multichannel EMG MAT logger.

    properties
        CompletedNormally (1,1) logical = false
    end

    properties (SetAccess = private)
        RawFile (1,1) string
        MetadataFile (1,1) string
        LoggedSamples (1,1) double = 0
        Closed (1,1) logical = false
    end

    properties (Access = private)
        Metadata (1,1) struct
        ChannelCount (1,1) double
        BufferCapacity (1,1) double
        BufferCount (1,1) double = 0
        TimeBuffer double
        VoltageBuffer double
        DurationBuffer double
        PhaseBuffer uint8
        MarkerBuffer uint8
        SequenceBuffer uint16
        StatusBuffer uint8
        RawLog = []
    end

    methods
        function obj = EMGSessionLogger(rawFile, metadataFile, metadata, ...
                channelCount, bufferCapacity)
            arguments
                rawFile (1,1) string
                metadataFile (1,1) string
                metadata (1,1) struct
                channelCount (1,1) double {mustBeInteger, mustBePositive}
                bufferCapacity (1,1) double {mustBeInteger, mustBePositive}
            end
            obj.RawFile = rawFile;
            obj.MetadataFile = metadataFile;
            obj.Metadata = metadata;
            obj.ChannelCount = channelCount;
            obj.BufferCapacity = bufferCapacity;
            obj.TimeBuffer = zeros(bufferCapacity, 1);
            obj.VoltageBuffer = zeros(bufferCapacity, channelCount);
            obj.DurationBuffer = zeros(bufferCapacity, 1);
            obj.PhaseBuffer = zeros(bufferCapacity, 1, 'uint8');
            obj.MarkerBuffer = zeros(bufferCapacity, 1, 'uint8');
            obj.SequenceBuffer = zeros(bufferCapacity, 1, 'uint16');
            obj.StatusBuffer = zeros(bufferCapacity, 1, 'uint8');
            save(obj.MetadataFile, 'metadata');
        end

        function append(obj, timeSeconds, voltages, setDurationSeconds, ...
                phaseCode, markerCode, options)
            arguments
                obj (1,1) EMGSessionLogger
                timeSeconds (1,1) double {mustBeFinite}
                voltages (1,:) double
                setDurationSeconds (1,1) double {mustBeNonnegative}
                phaseCode (1,1) uint8
                markerCode (1,1) uint8
                options.SequenceNumber (1,1) uint16 = uint16(0)
                options.DeviceStatus (1,1) uint8 = uint8(0)
            end
            if obj.Closed
                error('MyoDrum:LoggerClosed', ...
                    'Cannot append after the EMG session logger has closed.');
            end
            if numel(voltages) ~= obj.ChannelCount
                error('MyoDrum:InvalidEMGChannelCount', ...
                    'Expected %d EMG voltages but received %d.', ...
                    obj.ChannelCount, numel(voltages));
            end
            if ~ismember(phaseCode, uint8([1, 2]))
                error('MyoDrum:InvalidEMGPhase', ...
                    'EMG phase must be uint8(1) REST or uint8(2) FREE.');
            end
            if markerCode > 3
                error('MyoDrum:InvalidEMGMarker', ...
                    'EMG marker must be an integer from 0 through 3.');
            end

            obj.BufferCount = obj.BufferCount + 1;
            row = obj.BufferCount;
            obj.TimeBuffer(row) = timeSeconds;
            obj.VoltageBuffer(row, :) = voltages;
            obj.DurationBuffer(row) = setDurationSeconds;
            obj.PhaseBuffer(row) = phaseCode;
            obj.MarkerBuffer(row) = markerCode;
            obj.SequenceBuffer(row) = options.SequenceNumber;
            obj.StatusBuffer(row) = options.DeviceStatus;
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
                obj.TimeBuffer(sourceRows);
            obj.RawLog.emgVoltage_V(destinationRows, 1:obj.ChannelCount) = ...
                obj.VoltageBuffer(sourceRows, :);
            obj.RawLog.setDuration_s(destinationRows, 1) = ...
                obj.DurationBuffer(sourceRows);
            obj.RawLog.phase(destinationRows, 1) = ...
                obj.PhaseBuffer(sourceRows);
            obj.RawLog.marker(destinationRows, 1) = ...
                obj.MarkerBuffer(sourceRows);
            obj.RawLog.sequenceNumber(destinationRows, 1) = ...
                obj.SequenceBuffer(sourceRows);
            obj.RawLog.deviceStatus(destinationRows, 1) = ...
                obj.StatusBuffer(sourceRows);
            obj.LoggedSamples = obj.LoggedSamples + obj.BufferCount;
            obj.BufferCount = 0;
        end

        function updateMetadata(obj, fields)
            arguments
                obj (1,1) EMGSessionLogger
                fields (1,1) struct
            end
            names = fieldnames(fields);
            for fieldIndex = 1:numel(names)
                obj.Metadata.(names{fieldIndex}) = fields.(names{fieldIndex});
            end
        end

        function close(obj)
            if obj.Closed
                return;
            end
            obj.flush();
            metadata = obj.Metadata;
            metadata.endedAt = string(datetime('now', 'TimeZone', 'local'));
            metadata.loggedSamples = obj.LoggedSamples;
            metadata.completedNormally = obj.CompletedNormally;
            save(obj.MetadataFile, 'metadata');
            obj.Closed = true;
        end
    end
end
