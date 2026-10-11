function [hitIndices, replayedHits] = replayDetector( ...
        rawSessionFile, directions, kickCfg, sampleRateHz)
% REPLAYDETECTOR Replay a paired recorded stream through the kick detector.

    arguments
        rawSessionFile (1,1) string
        directions (1,2) struct
        kickCfg (1,1) struct
        sampleRateHz (1,1) double {mustBeFinite, mustBePositive}
    end

    requiredFields = ["time_s", "accelCalibrated_g", ...
        "gyroCorrected_rad_s", "kickHit"];
    recorded = load(rawSessionFile);
    for fieldName = requiredFields
        if ~isfield(recorded, fieldName)
            error('MyoDrum:InvalidReplaySession', ...
                'Recorded session is missing field %s: %s', ...
                fieldName, rawSessionFile);
        end
    end

    sampleCount = numel(recorded.time_s);
    if ~isequal(size(recorded.accelCalibrated_g), [sampleCount, 3, 2]) || ...
            ~isequal(size(recorded.gyroCorrected_rad_s), [sampleCount, 3, 2])
        error('MyoDrum:InvalidReplaySession', ...
            'Recorded session arrays have inconsistent dimensions: %s', ...
            rawSessionFile);
    end

    replayedHits = false(sampleCount, 2);
    detectors = [initializeKickDetector, initializeKickDetector];
    for sampleIndex = 1:sampleCount
        for sensorIndex = 1:2
            accelMps2 = recorded.accelCalibrated_g( ...
                sampleIndex, :, sensorIndex) * 9.80665;
            gyroRadPerSec = recorded.gyroCorrected_rad_s( ...
                sampleIndex, :, sensorIndex);
            [detectors(sensorIndex), replayedHits(sampleIndex, sensorIndex)] = ...
                updateKickDetector(detectors(sensorIndex), accelMps2, ...
                gyroRadPerSec, directions(sensorIndex), ...
                recorded.time_s(sampleIndex), sampleRateHz, kickCfg);
        end
    end

    hitIndices = cell(1, 2);
    for sensorIndex = 1:2
        hitIndices{sensorIndex} = find(replayedHits(:, sensorIndex));
    end
end
