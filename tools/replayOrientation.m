function result = replayOrientation(sessionId, params)
% REPLAYORIENTATION Re-run the gravity/azimuth estimator over a recorded session.
%
%   result = replayOrientation("session_20261010_145313")
%
% Uses the logged calibrated accelerometer, bias-corrected gyro and the real
% per-sample timestamps, together with the detector and calibrated stroke
% directions of that session, so the new orientation features can be judged
% offline without hardware.

    arguments
        sessionId (1,1) string
        params struct = struct
    end

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));

    rawFile = fullfile(root, 'data', 'raw', sessionId, 'dualKickIMU.mat');
    metadataFolder = fullfile(root, 'data', 'metadata', 'sessions', sessionId);
    recorded = load(rawFile);
    session = load(fullfile(metadataFolder, 'sessionMetadata.mat'), 'metadata');
    calibration = load(fullfile(metadataFolder, 'imuSessionCalibration.mat'), ...
        'calibration');
    kickCfg = myoDrumConfig(root).Kick;
    if isfield(session.metadata, 'config') && ...
            isfield(session.metadata.config, 'Kick')
        saved = session.metadata.config.Kick;
        names = fieldnames(saved);
        for nameIndex = 1:numel(names)
            kickCfg.(names{nameIndex}) = saved.(names{nameIndex});
        end
    end
    sampleRateHz = session.metadata.measuredSampleRateHz;

    time = recorded.time_s(:);
    sampleCount = numel(time);
    features = nan(sampleCount, 3, 2);   % azimuth, pitch, roll
    converged = false(sampleCount, 2);
    still = false(sampleCount, 2);
    biasRadPerSec = nan(sampleCount, 3, 2);
    hits = false(sampleCount, 2);
    armIndex = nan(sampleCount, 2);

    estimators = {GravityAzimuthEstimator(params), GravityAzimuthEstimator(params)};
    if ~isfield(params, 'UseStrokeAxis') || params.UseStrokeAxis
        for handIndex = 1:2
            estimators{handIndex}.setStrokeAxis( ...
                calibration.calibration.directions(handIndex).axis);
        end
    end
    detectors = [initializeKickDetector, initializeKickDetector];
    lastArmIndex = [NaN, NaN];

    freezeAfterHitSeconds = 0;
    freezeWhileArmed = false;
    if isfield(params, 'FreezeAfterHitSeconds')
        freezeAfterHitSeconds = params.FreezeAfterHitSeconds;
    end
    if isfield(params, 'FreezeWhileArmed')
        freezeWhileArmed = params.FreezeWhileArmed;
    end
    lastHitTime = [-inf, -inf];

    for sampleIndex = 1:sampleCount
        for handIndex = 1:2
            accelG = recorded.accelCalibrated_g(sampleIndex, :, handIndex);
            gyro = recorded.gyroCorrected_rad_s(sampleIndex, :, handIndex);
            % Same ordering as the live loop: detector first, then the
            % estimator (so the freeze uses the detector state).
            previousMode = detectors(handIndex).mode;
            [detectors(handIndex), hit] = updateKickDetector( ...
                detectors(handIndex), accelG * 9.80665, gyro, ...
                calibration.calibration.directions(handIndex), ...
                time(sampleIndex), sampleRateHz, kickCfg);
            if previousMode ~= 2 && detectors(handIndex).mode == 2
                lastArmIndex(handIndex) = sampleIndex;
            end
            if hit
                lastHitTime(handIndex) = time(sampleIndex);
            end
            freeze = (freezeWhileArmed && detectors(handIndex).mode == 2) || ...
                (time(sampleIndex) - lastHitTime(handIndex)) < freezeAfterHitSeconds;
            estimators{handIndex}.update(accelG, gyro, time(sampleIndex), freeze);
            if estimators{handIndex}.Initialized
                features(sampleIndex, :, handIndex) = ...
                    estimators{handIndex}.Features();
                converged(sampleIndex, handIndex) = ...
                    estimators{handIndex}.Converged;
                still(sampleIndex, handIndex) = estimators{handIndex}.Still;
                biasRadPerSec(sampleIndex, :, handIndex) = ...
                    estimators{handIndex}.BiasRadPerSec;
            end
            hits(sampleIndex, handIndex) = hit;
            if hit
                armIndex(sampleIndex, handIndex) = lastArmIndex(handIndex);
            end
        end
    end

    result.sessionId = sessionId;
    result.time = time;
    result.features = features;
    result.converged = converged;
    result.still = still;
    result.biasRadPerSec = biasRadPerSec;
    result.hits = hits;
    result.armIndex = armIndex;
    result.loggedHits = logical(recorded.kickHit);
    result.hitsMatchLog = isequal(hits, result.loggedHits);
    result.gyroRadPerSec = recorded.gyroCorrected_rad_s;
    result.accelG = recorded.accelCalibrated_g;
    result.legacyYawPitchRoll = recorded.yawPitchRoll_deg;
    result.sampleRateHz = sampleRateHz;
    if isfield(recorded, 'targetIndex')
        result.targetIndex = recorded.targetIndex;
    end
end
