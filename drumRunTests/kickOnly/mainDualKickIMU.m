function mainDualKickIMU(mode)
% MAINDUALKICKIMU Calibrate, orient, log, and play drums from both IMUs.
%
% IMU 1 at 0x68 and IMU 2 at 0x69 each run an independent copy of the
% proven kick detector. Six-pose accelerometer calibration is persistent;
% gyro bias is measured per session.
%
% Orientation comes from GravityAzimuthEstimator, one per stick, updated on
% every sample with the real timestamps. Modes:
%   "kick"        every detected hit plays KICK.
%   "three-drum"  either stick plays any calibrated drum. The default
%                 gravity-zones strategy uses pitch and roll only, avoiding
%                 unobservable MPU6050 yaw. Azimuth and hand-posture remain
%                 available as experimental and fallback strategies.
%   "three-drum-emg" repeats region calibration while recording the two
%                 IMUs and four EMG envelope channels with host timestamps.

    arguments
        mode (1,1) string {mustBeMember(mode, ...
            ["kick", "three-drum", "three-drum-emg"])} = "kick"
    end
    clc;
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(projectRoot, 'common'));
    addpath(fullfile(projectRoot, 'IMU_subsystem'));
    addpath(fullfile(projectRoot, 'calibration'));
    addpath(fullfile(projectRoot, 'classifiers', 'ruleBased'));
    addpath(fullfile(projectRoot, 'audio'));
    addpath(fullfile(projectRoot, 'EMG_subsystem'));
    cfg = myoDrumConfig(projectRoot);
    emgEnabled = mode == "three-drum-emg";
    threeDrumEnabled = mode ~= "kick";

    sessionId = "session_" + string(datetime('now', ...
        'Format', 'yyyyMMdd_HHmmss'));
    rawFolder = fullfile(cfg.Paths.RawData, sessionId);
    metadataFolder = fullfile(cfg.Paths.Metadata, 'sessions', sessionId);
    mkdir(rawFolder);
    mkdir(metadataFolder);
    rawFile = fullfile(rawFolder, 'dualKickIMU.mat');
    metadataFile = fullfile(metadataFolder, 'sessionMetadata.mat');

    metadata.sessionId = sessionId;
    metadata.startedAt = string(datetime('now', 'TimeZone', 'local'));
    metadata.mode = mode;
    if ~threeDrumEnabled
        metadata.target = "KICK";
    else
        metadata.target = "THREE_DRUMS";
    end
    metadata.config = cfg;
    metadata.rawDataFile = string(rawFile);
    metadata.calibrationFile = string(fullfile( ...
        metadataFolder, 'imuSessionCalibration.mat'));
    metadata.orientationEstimator = cfg.Orientation.EstimatorVersion;
    metadata.regionStrategy = cfg.Regions.Strategy;
    metadata.gyroRangeDps = cfg.IMU.GyroscopeRangeDps;
    metadata.completedNormally = false;
    save(metadataFile, 'metadata');

    fprintf('MyoDrum: calibrated two-IMU %s session\n', mode);
    fprintf('Session: %s\n', sessionId);
    fprintf('Connecting to %s on %s...\n', ...
        cfg.Hardware.Board, cfg.Hardware.COMPort);
    board = arduino(cfg.Hardware.COMPort, cfg.Hardware.Board, ...
        'Libraries', 'I2C');
    if emgEnabled
        for pin = cfg.EMG.AnalogPins.'
            configurePin(board, pin, 'AnalogInput');
        end
        metadata.emgChannelNames = cfg.EMG.ChannelNames;
        metadata.emgAnalogPins = cfg.EMG.AnalogPins;
        metadata.emgHands = cfg.EMG.Hands;
        metadata.timestampSource = ...
            "host tic/toc acquisition windows; not a device clock";
        metadata.acquisitionMethod = ...
            "MATLAB Arduino I2C plus sequential readVoltage";
    end
    sensors = setupMPU6050Pair( ...
        board, cfg.IMU.Addresses, cfg.IMU.SensorOutputRateHz, ...
        cfg.IMU.DLPFConfig, cfg.IMU.GyroscopeRangeDps);
    sampleRateHz = measureDualIMUReadRate(sensors, 25);
    metadata.measuredSampleRateHz = sampleRateHz;
    calibration = calibrateDualIMUSession( ...
        sensors, sampleRateHz, cfg, metadataFolder);
    metadata.gyroBiasPolicy = calibration.gyroBiasPolicy;
    metadata.gyroBiasSource = calibration.gyroBiasSource;
    save(metadataFile, 'metadata');

    % One continuous clock drives the estimators through warm-up,
    % calibration and play so their per-sample dt is always real.
    sessionClock = tic;
    estimators = createEstimators(cfg, calibration);
    warmUpEstimators(sensors, calibration, estimators, sessionClock, cfg);

    postureModel = struct;
    gravityRegionModel = struct;
    azimuthTracker = [];
    if threeDrumEnabled
        if cfg.Regions.Strategy == "gravity-zones"
            gravityRegionModel = loadOrCollectGravityRegionModel(sensors, ...
                calibration, estimators, sessionClock, sampleRateHz, ...
                cfg, metadataFolder, board, rawFolder, emgEnabled);
            metadata.gravityRegionModelFile = string(fullfile( ...
                metadataFolder, 'threeDrumGravityRegionModel.mat'));
            metadata.gravityRegionTrainingAccuracy = ...
                gravityRegionModel.trainingAccuracyByHand;
            metadata.gravityRegionMinimumSeparationDeg = ...
                gravityRegionModel.minimumSeparationDegByHand;
        elseif cfg.Regions.Strategy == "azimuth"
            [azimuthTracker, verification] = ...
                calibrateAzimuthLayout(sensors, calibration, estimators, ...
                sessionClock, sampleRateHz, cfg, metadataFolder);
            metadata.azimuthModelFile = string(fullfile( ...
                metadataFolder, 'threeDrumAzimuthModel.mat'));
            metadata.azimuthModelOk = [ ...
                azimuthTracker.Model.hands(1).ok, ...
                azimuthTracker.Model.hands(2).ok];
            metadata.azimuthDriftDegPerSec = [ ...
                azimuthTracker.Model.hands(1).driftDegPerSec, ...
                azimuthTracker.Model.hands(2).driftDegPerSec];
            metadata.verificationAccuracy = verification.accuracyByHand;
        else
            postureModel = loadOrCollectPostureModel(sensors, ...
                calibration, estimators, sessionClock, sampleRateHz, ...
                cfg, metadataFolder);
            metadata.postureModelFile = string(fullfile( ...
                metadataFolder, 'threeDrumPostureModel.mat'));
            metadata.postureCrashEnabled = [ ...
                postureModel.hands(1).crashEnabled, ...
                postureModel.hands(2).crashEnabled];
        end
        save(metadataFile, 'metadata');
        activeTargets = cfg.Targets.Names;
        activeAudioFiles = cfg.Targets.AudioFiles;
    else
        activeTargets = "KICK";
        activeAudioFiles = "kick.wav";
    end
    audioBanks = createDrumPlayerBanks(cfg.Paths.Audio, activeTargets, ...
        activeAudioFiles, cfg.Kick.AudioVoices);
    players = flattenPlayerBanks(audioBanks, activeTargets);
    detectors = [initializeKickDetector, initializeKickDetector];
    hitCount = zeros(1, 2);
    gyroSaturatedCount = zeros(1, 2);
    gyroSaturationDps = 0.98 * cfg.IMU.GyroscopeRangeDps;

    fig = createControlFigure(mode);
    logger = DualIMUSessionLogger(rawFile, metadataFile, metadata, ...
        cfg.Logging.BufferSamples, emgEnabled * numel(cfg.EMG.AnalogPins));
    cleanup = onCleanup(@()shutdownDualKick( ...
        logger, players, fig, cfg, metadataFolder));

    fprintf('\nREADY: %s mode.\n', upper(mode));
    if threeDrumEnabled
        if cfg.Regions.Strategy == "gravity-zones"
            fprintf(['Either stick plays %s. Change the stick pitch/roll ' ...
                'to select a drum; uncertain poses are silent.\n'], ...
                strjoin(cfg.Targets.Names, ' / '));
        elseif cfg.Regions.Strategy == "azimuth"
            fprintf(['Either stick plays the drum it is aimed at ' ...
                '(%s).\n'], strjoin(unique( ...
                cfg.Regions.CalibrationOrder, 'stable'), ' / '));
        else
            fprintf('LEFT hand plays %s, RIGHT hand plays %s.\n', ...
                cfg.Regions.HandDrums(1), cfg.Regions.HandDrums(2));
            fprintf(['Use the calibrated %s posture with either stick ' ...
                'for %s.\n'], cfg.Regions.PostureTarget, ...
                cfg.Regions.PostureTarget);
        end
    end
    fprintf('Live plotting is disabled. Press Q or close the control window to stop.\n\n');
    liveStartSeconds = toc(sessionClock);
    sampleNumber = 0;
    armGravity = nan(2, 3);
    armAzimuth = nan(1, 2);
    activeRegion = strings(1, 2);
    armedRegion = strings(1, 2);
    strokePeakGyro = zeros(2, 3);
    strokePeakGyroMagnitude = zeros(1, 2);
    liveReadStates = cell(1, 2);
    for sensorIndex = 1:2
        [~, liveReadStates{sensorIndex}] = ...
            readMPU6050Sample(sensors(sensorIndex));
    end
    previousCycleTime = NaN;

    while isgraphics(fig) && getappdata(fig, 'MyoDrumRunning')
        sampleNumber = sampleNumber + 1;
        clockNow = toc(sessionClock);
        timeNow = clockNow - liveStartSeconds;
        loopDurationSeconds = timeNow - previousCycleTime;
        previousCycleTime = timeNow;
        cycleRawAccel = zeros(1, 3, 2);
        cycleCalibratedAccel = zeros(1, 3, 2);
        cycleCorrectedGyro = zeros(1, 3, 2);
        cycleTemperature = zeros(1, 2);
        cycleYpr = zeros(1, 3, 2);
        cycleHit = false(1, 2);
        cycleDiagnostics = cell(1, 2);
        cycleLabels = strings(1, 2);
        cycleTargetIndex = zeros(1, 2, 'uint8');
        samples = cell(1, 2);
        cycleFrameValid = true(1, 2);
        imuReadWindow = [toc(sessionClock), NaN];

        % Acquire both hands back-to-back before doing any filtering,
        % plotting, logging, console output, or audio work. This minimizes
        % left/right sampling skew and makes simultaneous hits observable.
        for sensorIndex = 1:2
            [samples{sensorIndex}, liveReadStates{sensorIndex}] = ...
                readMPU6050Sample(sensors(sensorIndex), Mode="live", ...
                State=liveReadStates{sensorIndex});
            cycleFrameValid(sensorIndex) = samples{sensorIndex}.valid;
        end
        imuReadWindow(2) = toc(sessionClock);

        for sensorIndex = 1:2
            sample = samples{sensorIndex};
            calibratedAccelG = applyMPU6050AccelCalibration( ...
                sample.accelG, calibration.accel{sensorIndex});
            accelMps2 = calibratedAccelG * 9.80665;
            correctedGyroRad = deg2rad(sample.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));
            gravityBefore = estimators{sensorIndex}.GravityBody;
            modeBefore = detectors(sensorIndex).mode;

            if threeDrumEnabled && ...
                    cfg.Regions.Strategy == "gravity-zones"
                activeRegion(sensorIndex) = classifyGravityDrum( ...
                    gravityRegionModel, sensorIndex, gravityBefore, ...
                    MaximumDistance=cfg.Regions.MaximumNormalizedDistance, ...
                    AmbiguityMargin=cfg.Regions.AmbiguityMargin, ...
                    CurrentLabel=activeRegion(sensorIndex), ...
                    ExitDistance=cfg.Regions.GravityZoneExitDistance);
            end

            if ~sample.valid
                cycleRawAccel(1, :, sensorIndex) = sample.accelG;
                cycleCalibratedAccel(1, :, sensorIndex) = calibratedAccelG;
                cycleCorrectedGyro(1, :, sensorIndex) = correctedGyroRad;
                cycleTemperature(1, sensorIndex) = sample.temperatureC;
                continue;
            end

            [detectors(sensorIndex), hit, diagnostics] = ...
                updateKickDetector(detectors(sensorIndex), accelMps2, ...
                correctedGyroRad, calibration.directions(sensorIndex), ...
                timeNow, sampleRateHz, cfg.Kick);

            signedStrokeRate = calibration.directions(sensorIndex).sign * ...
                correctedGyroRad(calibration.directions(sensorIndex).axis);
            gyroMagnitude = norm(correctedGyroRad);
            if modeBefore == 2 && signedStrokeRate > 0 && ...
                    gyroMagnitude > strokePeakGyroMagnitude(sensorIndex)
                strokePeakGyro(sensorIndex, :) = correctedGyroRad;
                strokePeakGyroMagnitude(sensorIndex) = gyroMagnitude;
            end

            % Gravity at the moment the downstroke armed is the clean,
            % pre-impact posture of the stick.
            if detectors(sensorIndex).mode == 2 && modeBefore ~= 2
                armGravity(sensorIndex, :) = gravityBefore;
                armAzimuth(sensorIndex) = estimators{sensorIndex}.AzimuthDeg();
                armedRegion(sensorIndex) = activeRegion(sensorIndex);
                strokePeakGyro(sensorIndex, :) = correctedGyroRad;
                strokePeakGyroMagnitude(sensorIndex) = gyroMagnitude;
            end

            cycleRawAccel(1, :, sensorIndex) = sample.accelG;
            cycleCalibratedAccel(1, :, sensorIndex) = ...
                calibratedAccelG;
            cycleCorrectedGyro(1, :, sensorIndex) = ...
                correctedGyroRad;
            cycleTemperature(1, sensorIndex) = sample.temperatureC;
            cycleHit(1, sensorIndex) = hit;
            cycleDiagnostics{sensorIndex} = diagnostics;
            if any(abs(sample.gyroDps) >= gyroSaturationDps)
                gyroSaturatedCount(sensorIndex) = ...
                    gyroSaturatedCount(sensorIndex) + 1;
            end
            if hit
                if mode == "kick"
                    cycleLabels(sensorIndex) = "KICK";
                elseif cfg.Regions.Strategy == "gravity-zones"
                    cycleLabels(sensorIndex) = classifyGravityDrum( ...
                        gravityRegionModel, sensorIndex, ...
                        armGravity(sensorIndex, :), ...
                        MaximumDistance= ...
                            cfg.Regions.MaximumNormalizedDistance, ...
                        AmbiguityMargin=cfg.Regions.AmbiguityMargin, ...
                        CurrentLabel=armedRegion(sensorIndex), ...
                        ExitDistance=cfg.Regions.GravityZoneExitDistance, ...
                        GyroPeakVector=strokePeakGyro(sensorIndex, :), ...
                        MaximumGyroAngleDeg= ...
                            cfg.Regions.GravityZoneMaximumGyroAngleDeg);
                elseif cfg.Regions.Strategy == "azimuth"
                    azimuthForHit = estimators{sensorIndex}.AzimuthDeg();
                    if cfg.Regions.AzimuthFeatureTime == "arm" && ...
                            isfinite(armAzimuth(sensorIndex))
                        azimuthForHit = armAzimuth(sensorIndex);
                    end
                    cycleLabels(sensorIndex) = azimuthTracker.classify( ...
                        sensorIndex, azimuthForHit, true);
                else
                    feature = postureFeature(cfg, armGravity(sensorIndex, :), ...
                        gravityBefore);
                    [cycleLabels(sensorIndex), confidence] = classifyPosture( ...
                        postureModel, sensorIndex, feature);
                    if cycleLabels(sensorIndex) == cfg.Regions.PostureTarget
                        fprintf('%s %s posture confidence %.2f\n', ...
                            cfg.IMU.HandNames(sensorIndex), ...
                            cycleLabels(sensorIndex), confidence);
                    end
                end
                if cycleLabels(sensorIndex) ~= ""
                    cycleTargetIndex(sensorIndex) = uint8(find( ...
                        cfg.Targets.Names == cycleLabels(sensorIndex), 1));
                end
            end
        end

        % Merge only identical labels in the same acquisition cycle. Two
        % different drums struck together retain two independent sounds.
        labelsToPlay = unique(cycleLabels(cycleLabels ~= ""), 'stable');
        for label = labelsToPlay
            audioBanks = playDrumLabel(audioBanks, label);
        end
        for sensorIndex = find(cycleHit)
            hitCount(sensorIndex) = hitCount(sensorIndex) + 1;
            diagnostics = cycleDiagnostics{sensorIndex};
            fprintf('%s %s %d | rate=%+.2f rad/s | jerk=%.2f m/s^2\n', ...
                cfg.IMU.HandNames(sensorIndex), cycleLabels(sensorIndex), ...
                hitCount(sensorIndex), diagnostics.signedRateRadPerSec, ...
                diagnostics.accelStepMps2);
        end

        % Orientation is not needed to recognise a hit. Update it after the
        % audio so it cannot delay a detected strike.
        for sensorIndex = 1:2
            if cycleFrameValid(sensorIndex)
                estimators{sensorIndex}.update( ...
                    cycleCalibratedAccel(1, :, sensorIndex), ...
                    cycleCorrectedGyro(1, :, sensorIndex), clockNow);
            end
            cycleYpr(1, :, sensorIndex) = estimators{sensorIndex}.Features();
        end

        emgVoltages = zeros(1, 0);
        emgReadWindow = [NaN, NaN];
        if emgEnabled
            [emgVoltages, emgReadWindow] = readEMGSet( ...
                @(pin)readVoltage(board, pin), cfg.EMG.AnalogPins, ...
                sessionClock);
        end
        if getappdata(fig, 'MyoDrumRecenterRequested')
            for sensorIndex = 1:2
                estimators{sensorIndex}.recenter();
            end
            setappdata(fig, 'MyoDrumRecenterRequested', false);
            if ~isempty(azimuthTracker)
                azimuthTracker.reset();
            end
            fprintf(['Azimuth recentered. Hold both sticks in the same ' ...
                'neutral pose you used at warm-up.\n']);
        end

        logger.append(timeNow, cycleRawAccel, cycleCalibratedAccel, ...
            cycleCorrectedGyro, cycleTemperature, cycleYpr, cycleHit, ...
            cycleFrameValid, loopDurationSeconds, cycleTargetIndex, ...
            imuReadWindow, emgVoltages, emgReadWindow);
        if mod(sampleNumber, cfg.Orientation.ControlPollEverySamples) == 0
            drawnow limitrate;
        end
    end

    if sampleNumber > 0
        fprintf(['Gyro at %.0f%% of full scale (+/-%d dps) in %.1f%% (%s) ' ...
            'and %.1f%% (%s) of samples.\n'], 98, ...
            cfg.IMU.GyroscopeRangeDps, ...
            100 * gyroSaturatedCount(1) / sampleNumber, ...
            cfg.IMU.HandNames(1), ...
            100 * gyroSaturatedCount(2) / sampleNumber, ...
            cfg.IMU.HandNames(2));
    end
    logger.CompletedNormally = true;
    clear cleanup; % closes logger and releases hardware-related resources
end

function estimators = createEstimators(cfg, calibration)
    estimators = cell(1, 2);
    for sensorIndex = 1:2
        estimators{sensorIndex} = GravityAzimuthEstimator( ...
            cfg.Orientation.EstimatorOverrides);
        estimators{sensorIndex}.setStrokeAxis( ...
            calibration.directions(sensorIndex).axis);
    end
end

function warmUpEstimators(sensors, calibration, estimators, sessionClock, cfg)
    fprintf('\nORIENTATION WARM-UP\n');
    fprintf(['Hold both sticks still in a comfortable neutral playing pose ' ...
        '(at least %.0f s).\n'], cfg.Orientation.WarmupSeconds);
    startSeconds = toc(sessionClock);
    converged = false;
    while true
        clockNow = toc(sessionClock);
        for sensorIndex = 1:2
            sample = readMPU6050Sample(sensors(sensorIndex));
            accelG = applyMPU6050AccelCalibration( ...
                sample.accelG, calibration.accel{sensorIndex});
            gyroRad = deg2rad(sample.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));
            estimators{sensorIndex}.update(accelG, gyroRad, clockNow);
        end
        elapsed = clockNow - startSeconds;
        converged = estimators{1}.Converged && estimators{2}.Converged;
        if elapsed >= cfg.Orientation.WarmupSeconds && converged
            break;
        end
        if elapsed >= cfg.Orientation.MaxWarmupSeconds
            break;
        end
    end
    for sensorIndex = 1:2
        estimators{sensorIndex}.recenter();
    end
    if converged
        fprintf('Orientation converged.\n');
    else
        warning('MyoDrum:OrientationNotConverged', ...
            ['Orientation did not settle within %.0f s. Keep the sticks ' ...
             'still next time; posture calibration may be less reliable.'], ...
            cfg.Orientation.MaxWarmupSeconds);
    end
    fprintf('Azimuth is relative and will drift (no magnetometer); tilt will not.\n');
end

function model = loadOrCollectGravityRegionModel(sensors, calibration, ...
        estimators, sessionClock, sampleRateHz, cfg, metadataFolder, ...
        board, rawFolder, emgEnabled)
% Fit three pitch/roll regions per hand. Every hand can select every drum.
    persistentFile = fullfile(cfg.Paths.Metadata, 'calibration', ...
        'threeDrumGravityRegionModel.mat');
    sessionFile = fullfile(metadataFolder, 'threeDrumGravityRegionModel.mat');
    if ~emgEnabled && ~cfg.Calibration.ForceRegionCalibration && ...
            isfile(persistentFile)
        saved = load(persistentFile, 'model', 'estimatorVersion');
        if gravityRegionModelIsCurrent(saved, cfg)
            model = saved.model;
            fprintf('Loaded saved gravity-region model:\n  %s\n', persistentFile);
            printGravityRegionSummary(model, cfg);
            save(sessionFile, 'model');
            return;
        end
        fprintf(['Saved gravity-region model has a different target list ' ...
            'or estimator; recalibrating.\n']);
    end

    model = [];
    gestureRecording = initializeGestureRecording(numel(cfg.EMG.AnalogPins));
    for attempt = 1:cfg.Regions.GravityZoneCalibrationAttempts
        gravityVectors = zeros(0, 3);
        gyroPeakVectors = zeros(0, 3);
        hands = zeros(0, 1);
        labels = strings(0, 1);
        fprintf('\nTHREE-DRUM PITCH/ROLL CALIBRATION (attempt %d/%d)\n', ...
            attempt, cfg.Regions.GravityZoneCalibrationAttempts);
        fprintf(['Aim or tilt each stick distinctly toward the printed drum. ' ...
            'The horizontal position alone is not observable.\n']);
        for hand = 1:2
            for target = cfg.Targets.Names
                input(sprintf(['\n%s hand: play %s %d times. Hold the same ' ...
                    'aim/tilt for every stroke, then press Enter...'], ...
                    cfg.IMU.HandNames(hand), target, ...
                    cfg.Targets.CalibrationHitsPerTargetPerHand), 's');
                [collectedGravity, collectedGyro, trialRecording] = ...
                    collectGravityRegionHits(sensors, calibration, ...
                    estimators, sessionClock, sampleRateHz, cfg, hand, ...
                    cfg.Targets.CalibrationHitsPerTargetPerHand, board, ...
                    emgEnabled);
                if emgEnabled
                    trialRecording.handIndex(:) = uint8(hand);
                    trialRecording.targetIndex(:) = uint8(find( ...
                        cfg.Targets.Names == target, 1));
                    trialRecording.attempt(:) = uint8(attempt);
                    gestureRecording = appendGestureRecording( ...
                        gestureRecording, trialRecording);
                    % Checkpoint after every labelled trial so an operator
                    % stop does not discard all earlier gesture data.
                    save(fullfile(rawFolder, ...
                        'imuEmgGestureCalibration.mat'), ...
                        'gestureRecording', '-v7.3');
                end
                gravityVectors = [gravityVectors; collectedGravity]; %#ok<AGROW>
                gyroPeakVectors = [gyroPeakVectors; collectedGyro]; %#ok<AGROW>
                hands = [hands; repmat(hand, ...
                    size(collectedGravity, 1), 1)]; %#ok<AGROW>
                labels = [labels; repmat(target, ...
                    size(collectedGravity, 1), 1)]; %#ok<AGROW>
            end
        end
        model = buildGravityDrumModel(gravityVectors, gyroPeakVectors, ...
            hands, labels, cfg.Targets.Names, ...
            MinGravitySpreadDeg=cfg.Regions.GravityZoneMinSpreadDeg, ...
            MinGyroSpreadDeg=cfg.Regions.GravityZoneMinGyroSpreadDeg, ...
            MinSeparationDeg=cfg.Regions.GravityZoneMinSeparationDeg, ...
            MinTrainingAccuracy=cfg.Regions.GravityZoneMinAccuracy);
        printGravityRegionSummary(model, cfg);
        if all(model.okByHand)
            break;
        end
        if attempt < cfg.Regions.GravityZoneCalibrationAttempts
            fprintf(['Calibration regions overlap. Repeat with more distinct ' ...
                'stick tilts for SNARE, KICK and CRASH.\n']);
        end
    end
    if ~all(model.okByHand)
        failed = strjoin(cfg.IMU.HandNames(~model.okByHand), ', ');
        error('MyoDrum:GravityRegionCalibrationFailed', ...
            ['Pitch/roll regions failed validation for %s. No unreliable ' ...
             'drum map will be used. Increase the difference between poses.'], ...
            failed);
    end
    estimatorVersion = cfg.Orientation.EstimatorVersion; %#ok<NASGU>
    persistentFolder = fileparts(persistentFile);
    if ~isfolder(persistentFolder)
        mkdir(persistentFolder);
    end
    save(persistentFile, 'model', 'estimatorVersion');
    save(sessionFile, 'model');
    if emgEnabled
        gestureRecording.channelNames = cfg.EMG.ChannelNames;
        gestureRecording.analogPins = cfg.EMG.AnalogPins;
        gestureRecording.targetNames = cfg.Targets.Names;
        gestureRecording.timestampSource = ...
            "host tic/toc acquisition windows; not a device clock";
        save(fullfile(rawFolder, 'imuEmgGestureCalibration.mat'), ...
            'gestureRecording', '-v7.3');
    end
end

function current = gravityRegionModelIsCurrent(saved, cfg)
    current = isfield(saved, 'model') && isstruct(saved.model) && ...
        isfield(saved, 'estimatorVersion') && ...
        string(saved.estimatorVersion) == cfg.Orientation.EstimatorVersion && ...
        isfield(saved.model, 'schemaVersion') && ...
        saved.model.schemaVersion == "gravity-vector-drums-1" && ...
        isfield(saved.model, 'targetNames') && ...
        isequal(saved.model.targetNames, cfg.Targets.Names) && ...
        isfield(saved.model, 'okByHand') && all(saved.model.okByHand);
end

function [gravityVectors, gyroPeakVectors, recording] = ...
        collectGravityRegionHits( ...
        sensors, calibration, ...
        estimators, sessionClock, sampleRateHz, cfg, hand, hitCountTarget, ...
        board, emgEnabled)
    gravityVectors = zeros(hitCountTarget, 3);
    gyroPeakVectors = zeros(hitCountTarget, 3);
    detector = initializeKickDetector;
    armedGravity = [NaN, NaN, NaN];
    peakGyro = [0, 0, 0];
    peakGyroMagnitude = 0;
    accepted = 0;
    recording = initializeGestureRecording(numel(cfg.EMG.AnalogPins));
    localTimer = tic;
    while accepted < hitCountTarget
        imuReadWindow = [toc(sessionClock), NaN];
        samples = cell(1, 2);
        for sensorIndex = 1:2
            samples{sensorIndex} = readMPU6050Sample(sensors(sensorIndex));
        end
        imuReadWindow(2) = toc(sessionClock);
        clockNow = toc(sessionClock);
        localTime = toc(localTimer);
        cycleRawAccel = nan(1, 3, 2);
        cycleCalibratedAccel = nan(1, 3, 2);
        cycleCorrectedGyro = nan(1, 3, 2);
        cycleFrameValid = false(1, 2);
        cycleHit = false(1, 2);
        for sensorIndex = 1:2
            accelG = applyMPU6050AccelCalibration( ...
                samples{sensorIndex}.accelG, calibration.accel{sensorIndex});
            gyroRad = deg2rad(samples{sensorIndex}.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));
            cycleRawAccel(1, :, sensorIndex) = samples{sensorIndex}.accelG;
            cycleCalibratedAccel(1, :, sensorIndex) = accelG;
            cycleCorrectedGyro(1, :, sensorIndex) = gyroRad;
            cycleFrameValid(sensorIndex) = samples{sensorIndex}.valid;
            hit = false;
            if sensorIndex == hand
                gravityBefore = estimators{sensorIndex}.GravityBody;
                modeBefore = detector.mode;
                [detector, hit] = updateKickDetector(detector, ...
                    accelG * 9.80665, gyroRad, ...
                    calibration.directions(sensorIndex), localTime, ...
                    sampleRateHz, cfg.Kick);
                if detector.mode == 2 && modeBefore ~= 2
                    armedGravity = gravityBefore;
                    peakGyro = gyroRad;
                    peakGyroMagnitude = norm(gyroRad);
                elseif modeBefore == 2
                    signedRate = calibration.directions(sensorIndex).sign * ...
                        gyroRad(calibration.directions(sensorIndex).axis);
                    if signedRate > 0 && norm(gyroRad) > peakGyroMagnitude
                        peakGyro = gyroRad;
                        peakGyroMagnitude = norm(gyroRad);
                    end
                end
            end
            if hit && all(isfinite(armedGravity)) && peakGyroMagnitude > 0
                accepted = accepted + 1;
                gravityVectors(accepted, :) = armedGravity;
                gyroPeakVectors(accepted, :) = peakGyro;
                fprintf(['  accepted %d/%d | gravity [%+.2f %+.2f %+.2f] ' ...
                    '| peak gyro %.2f rad/s\n'], accepted, hitCountTarget, ...
                    armedGravity, peakGyroMagnitude);
            end
            cycleHit(sensorIndex) = hit;
            estimators{sensorIndex}.update(accelG, gyroRad, clockNow);
        end
        if emgEnabled
            [emgVoltages, emgReadWindow] = readEMGSet( ...
                @(pin)readVoltage(board, pin), cfg.EMG.AnalogPins, ...
                sessionClock);
            row = size(recording.time_s, 1) + 1;
            recording.time_s(row, 1) = clockNow;
            recording.imuReadStart_s(row, 1) = imuReadWindow(1);
            recording.imuReadEnd_s(row, 1) = imuReadWindow(2);
            recording.imuTimestamp_s(row, 1) = mean(imuReadWindow);
            recording.accelRaw_g(row, :, :) = cycleRawAccel;
            recording.accelCalibrated_g(row, :, :) = cycleCalibratedAccel;
            recording.gyroCorrected_rad_s(row, :, :) = ...
                cycleCorrectedGyro;
            recording.frameValid(row, :) = cycleFrameValid;
            recording.kickHit(row, :) = cycleHit;
            recording.emgVoltage_V(row, :) = emgVoltages;
            recording.emgReadStart_s(row, 1) = emgReadWindow(1);
            recording.emgReadEnd_s(row, 1) = emgReadWindow(2);
            recording.emgTimestamp_s(row, 1) = mean(emgReadWindow);
            recording.handIndex(row, 1) = uint8(0);
            recording.targetIndex(row, 1) = uint8(0);
            recording.attempt(row, 1) = uint8(0);
        end
    end
end

function recording = initializeGestureRecording(channelCount)
    recording.time_s = zeros(0, 1);
    recording.imuReadStart_s = zeros(0, 1);
    recording.imuReadEnd_s = zeros(0, 1);
    recording.imuTimestamp_s = zeros(0, 1);
    recording.accelRaw_g = zeros(0, 3, 2);
    recording.accelCalibrated_g = zeros(0, 3, 2);
    recording.gyroCorrected_rad_s = zeros(0, 3, 2);
    recording.frameValid = false(0, 2);
    recording.kickHit = false(0, 2);
    recording.emgVoltage_V = zeros(0, channelCount);
    recording.emgReadStart_s = zeros(0, 1);
    recording.emgReadEnd_s = zeros(0, 1);
    recording.emgTimestamp_s = zeros(0, 1);
    recording.handIndex = zeros(0, 1, 'uint8');
    recording.targetIndex = zeros(0, 1, 'uint8');
    recording.attempt = zeros(0, 1, 'uint8');
end

function combined = appendGestureRecording(combined, trial)
    fields = fieldnames(combined);
    for fieldIndex = 1:numel(fields)
        name = fields{fieldIndex};
        combined.(name) = cat(1, combined.(name), trial.(name));
    end
end

function printGravityRegionSummary(model, cfg)
    for hand = 1:2
        fprintf(['  %s: self-accuracy %.0f%% | minimum centre spacing ' ...
            '%.0f deg | %s\n'], cfg.IMU.HandNames(hand), ...
            100 * model.trainingAccuracyByHand(hand), ...
            model.minimumSeparationDegByHand(hand), ...
            ternary(model.okByHand(hand), "OK", "NOT SEPARABLE"));
    end
end

function model = loadOrCollectPostureModel(sensors, calibration, ...
        estimators, sessionClock, sampleRateHz, cfg, metadataFolder)
    persistentFile = fullfile(cfg.Paths.Metadata, 'calibration', ...
        'threeDrumPostureModel.mat');
    sessionFile = fullfile(metadataFolder, 'threeDrumPostureModel.mat');
    if ~cfg.Calibration.ForceRegionCalibration && isfile(persistentFile)
        saved = load(persistentFile, 'model', 'estimatorVersion');
        if postureModelIsCurrent(saved, cfg)
            model = saved.model;
            fprintf('Loaded saved posture model:\n  %s\n', persistentFile);
            printPostureSummary(model, cfg);
            save(sessionFile, 'model');
            return;
        end
        fprintf(['Saved posture model was made with a different drum ' ...
            'mapping or estimator; recalibrating.\n']);
    end
    model = collectPostureCalibration(sensors, calibration, estimators, ...
        sessionClock, sampleRateHz, cfg);
    estimatorVersion = cfg.Orientation.EstimatorVersion; %#ok<NASGU>
    persistentFolder = fileparts(persistentFile);
    if ~isfolder(persistentFolder)
        mkdir(persistentFolder);
    end
    save(persistentFile, 'model', 'estimatorVersion');
    save(sessionFile, 'model');
end

function current = postureModelIsCurrent(saved, cfg)
    current = isfield(saved, 'model') && isstruct(saved.model) && ...
        isfield(saved, 'estimatorVersion') && ...
        string(saved.estimatorVersion) == cfg.Orientation.EstimatorVersion && ...
        isfield(saved.model, 'schemaVersion') && ...
        saved.model.schemaVersion == "posture-1" && ...
        isequal(saved.model.handDrumNames, cfg.Regions.HandDrums) && ...
        saved.model.crashName == cfg.Regions.PostureTarget;
end

function model = collectPostureCalibration(sensors, calibration, ...
        estimators, sessionClock, sampleRateHz, cfg)
    handDrums = cfg.Regions.HandDrums;
    crashName = cfg.Regions.PostureTarget;
    hitsPerClass = cfg.Targets.CalibrationHitsPerTargetPerHand;

    fprintf('\nPOSTURE CALIBRATION\n');
    fprintf(['LEFT plays %s and RIGHT plays %s by hand. Only %s needs a ' ...
        'posture.\n'], handDrums(1), handDrums(2), crashName);
    fprintf(['Make the %s posture clearly different from your normal ' ...
        'playing pose (for example raise or tilt the stick much further).\n'], ...
        crashName);

    perHand = struct('vectors', {zeros(0, 3), zeros(0, 3)}, ...
        'labels', {strings(0, 1), strings(0, 1)});
    needsCollection = [true, true];
    model = [];
    for attempt = 1:cfg.Regions.PostureCalibrationAttempts
        for hand = find(needsCollection)
            perHand(hand).vectors = zeros(0, 3);
            perHand(hand).labels = strings(0, 1);
            for className = [handDrums(hand), crashName]
                input(sprintf(['\n%s hand: play %s %d times with the %s ' ...
                    'posture. Press Enter to start...'], ...
                    cfg.IMU.HandNames(hand), className, hitsPerClass, ...
                    className), 's');
                collected = collectHits(sensors, calibration, estimators, ...
                    sessionClock, sampleRateHz, cfg, hand, hitsPerClass);
                perHand(hand).vectors = [perHand(hand).vectors; collected];
                perHand(hand).labels = [perHand(hand).labels; ...
                    repmat(className, size(collected, 1), 1)];
            end
        end
        vectors = [perHand(1).vectors; perHand(2).vectors];
        labels = [perHand(1).labels; perHand(2).labels];
        hands = [ones(numel(perHand(1).labels), 1); ...
            2 * ones(numel(perHand(2).labels), 1)];
        model = buildPostureModel(vectors, hands, labels, handDrums, ...
            crashName, MinSpreadDeg = cfg.Regions.PostureMinSpreadDeg, ...
            MinSeparationDeg = cfg.Regions.PostureMinSeparationDeg, ...
            MinTrainingAccuracy = cfg.Regions.PostureMinAccuracy, ...
            MaxNormalizedDistance = ...
            cfg.Regions.PostureMaxNormalizedDistance, ...
            MarginNormalized = cfg.Regions.PostureMarginNormalized);
        printPostureSummary(model, cfg);
        needsCollection = ~[model.hands.crashEnabled];
        if ~any(needsCollection)
            break;
        end
        if attempt < cfg.Regions.PostureCalibrationAttempts
            fprintf(['Repeating calibration for: %s. Make the %s posture ' ...
                'more different.\n'], ...
                strjoin(cfg.IMU.HandNames(needsCollection), ', '), crashName);
        end
    end
    for hand = find(needsCollection)
        warning('MyoDrum:CrashPostureDisabled', ...
            ['%s hand: %s posture was not separable (%s). That hand will ' ...
             'always play %s.'], cfg.IMU.HandNames(hand), crashName, ...
            model.hands(hand).reason, handDrums(hand));
    end
end

function vectors = collectHits(sensors, calibration, estimators, ...
        sessionClock, sampleRateHz, cfg, hand, hitCountTarget)
    vectors = zeros(hitCountTarget, 3);
    detector = initializeKickDetector;
    armGravity = nan(1, 3);
    accepted = 0;
    localTimer = tic;
    while accepted < hitCountTarget
        samples = cell(1, 2);
        for sensorIndex = 1:2
            samples{sensorIndex} = readMPU6050Sample(sensors(sensorIndex));
        end
        clockNow = toc(sessionClock);
        localTime = toc(localTimer);
        for sensorIndex = 1:2
            accelG = applyMPU6050AccelCalibration( ...
                samples{sensorIndex}.accelG, calibration.accel{sensorIndex});
            gyroRad = deg2rad(samples{sensorIndex}.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));
            hit = false;
            if sensorIndex == hand
                gravityBefore = estimators{sensorIndex}.GravityBody;
                modeBefore = detector.mode;
                [detector, hit] = updateKickDetector(detector, ...
                    accelG * 9.80665, gyroRad, ...
                    calibration.directions(sensorIndex), localTime, ...
                    sampleRateHz, cfg.Kick);
                if detector.mode == 2 && modeBefore ~= 2
                    armGravity = gravityBefore;
                end
            end
            if hit
                accepted = accepted + 1;
                vectors(accepted, :) = postureFeature( ...
                    cfg, armGravity, gravityBefore);
                fprintf('  accepted %d/%d\n', accepted, hitCountTarget);
            end
            estimators{sensorIndex}.update(accelG, gyroRad, clockNow);
        end
    end
end

function feature = postureFeature(cfg, armGravity, currentGravity)
    if cfg.Regions.PostureFeatureTime == "arm" && all(isfinite(armGravity))
        feature = armGravity;
    else
        feature = currentGravity;
    end
end

function printPostureSummary(model, cfg)
    for hand = 1:2
        h = model.hands(hand);
        fprintf(['  %s hand: %s vs %s | separation %.0f deg | ' ...
            'self-accuracy %.0f%% | %s posture %s\n'], ...
            cfg.IMU.HandNames(hand), model.handDrumNames(hand), ...
            model.crashName, h.separationDeg, 100 * h.trainingAccuracy, ...
            model.crashName, ternary(h.crashEnabled, "ENABLED", ...
            "disabled: " + h.reason));
    end
end

function [tracker, verification] = calibrateAzimuthLayout(sensors, ...
        calibration, estimators, sessionClock, sampleRateHz, cfg, ...
        metadataFolder)
% Interleaved calibration (so drift is identifiable), fit, then a prompted
% verification pass that measures real accuracy before play begins. The two
% hands alternate prompts so both share the same time window: heading drift
% then affects them equally and the model refers to the same moment.
    targetNames = unique(cfg.Regions.CalibrationOrder, 'stable');
    rounds = cfg.Regions.CalibrationRounds;
    sequence = repmat(cfg.Regions.CalibrationOrder, 1, rounds);

    fprintf('\nDRUM POSITION CALIBRATION\n');
    fprintf(['Stand as you will play. Either stick plays the drum it is ' ...
        'aimed at: %s.\n'], strjoin(targetNames, ' / '));
    fprintf(['The prompts alternate LEFT and RIGHT stick (%d hits each). ' ...
        'Hit the prompted drum with the prompted stick.\n'], numel(sequence));

    model = [];
    for attempt = 1:cfg.Regions.AzimuthCalibrationAttempts
        input(sprintf('\nPress Enter to start position calibration...'), 's');
        hits = collectPairedSequence(sensors, calibration, estimators, ...
            sessionClock, sampleRateHz, cfg, {sequence, sequence}, ...
            "calibration", []);
        az = [hits(1).azimuthDeg; hits(2).azimuthDeg];
        tm = [hits(1).clockSeconds; hits(2).clockSeconds];
        hd = [ones(numel(sequence), 1); 2 * ones(numel(sequence), 1)];
        lb = [sequence(:); sequence(:)];
        model = buildAzimuthModel(az, tm, hd, lb, targetNames, ...
            MinSpreadDeg = cfg.Regions.AzimuthMinSpreadDeg, ...
            MinSeparationDeg = cfg.Regions.AzimuthMinSeparationDeg, ...
            MinAccuracy = cfg.Regions.AzimuthMinAccuracy);
        printAzimuthSummary(model, cfg);
        if all([model.hands.ok])
            break;
        end
        if attempt < cfg.Regions.AzimuthCalibrationAttempts
            fprintf(['Positions were not clearly separated. Repeating: aim ' ...
                'at clearly different places for each drum.\n']);
        end
    end
    for hand = find(~[model.hands.ok])
        warning('MyoDrum:AzimuthLayoutUnreliable', ...
            ['%s hand: drum positions overlap (%s). Expect mistakes; ' ...
             'spread the drums further apart or aim more deliberately.'], ...
            cfg.IMU.HandNames(hand), model.hands(hand).reason);
    end

    tracker = DrumAzimuthTracker(model, cfg.Regions.Tracker);
    verification = verifyAzimuthLayout(sensors, calibration, estimators, ...
        sessionClock, sampleRateHz, cfg, tracker, targetNames);
    save(fullfile(metadataFolder, 'threeDrumAzimuthModel.mat'), ...
        'model', 'verification');
end

function verification = verifyAzimuthLayout(sensors, calibration, ...
        estimators, sessionClock, sampleRateHz, cfg, tracker, targetNames)
    count = cfg.Regions.VerifyHitsPerHand;
    verification.accuracyByHand = [NaN, NaN];
    verification.confusion = zeros(numel(targetNames), numel(targetNames), 2);
    if count <= 0
        return;
    end
    perTarget = ceil(count / numel(targetNames));
    sequences = cell(1, 2);
    for hand = 1:2
        sequence = repmat(targetNames, 1, perTarget);
        sequence = sequence(randperm(numel(sequence)));
        sequences{hand} = sequence(1:count);
    end
    fprintf(['\nVERIFICATION: play the prompted drums with the prompted ' ...
        'stick (nothing here is used for training).\n']);
    input('Press Enter to start verification...', 's');
    hits = collectPairedSequence(sensors, calibration, estimators, ...
        sessionClock, sampleRateHz, cfg, sequences, "verify", tracker);
    for hand = 1:2
        predicted = hits(hand).predicted(:);
        expected = sequences{hand}(:);
        correct = predicted == expected;
        verification.accuracyByHand(hand) = mean(correct);
        for k = 1:numel(expected)
            row = find(targetNames == expected(k));
            column = find(targetNames == predicted(k));
            verification.confusion(row, column, hand) = ...
                verification.confusion(row, column, hand) + 1;
        end
        fprintf('  %s hand: %.0f%% correct (%d of %d)\n', ...
            cfg.IMU.HandNames(hand), 100 * mean(correct), sum(correct), ...
            numel(correct));
    end
    fprintf('Verification accuracy: LEFT %.0f%%, RIGHT %.0f%%.\n', ...
        100 * verification.accuracyByHand);
    if any(verification.accuracyByHand < cfg.Regions.VerifyMinAccuracy)
        warning('MyoDrum:LowVerificationAccuracy', ...
            ['Verification accuracy is below %.0f%%. Restart to ' ...
             'recalibrate, and keep the drums well apart.'], ...
            100 * cfg.Regions.VerifyMinAccuracy);
    end
end

function hits = collectPairedSequence(sensors, calibration, estimators, ...
        sessionClock, sampleRateHz, cfg, sequences, phase, tracker)
% Prompt the two sticks alternately and record, for each accepted hit, the
% heading at the moment that stick's downstroke armed (the clean, pre-impact
% aim). Hits from the stick that was not prompted are ignored. When a
% tracker is given each accepted hit is also classified (and may update the
% tracker's drift offset), exactly as in play.
    totals = [numel(sequences{1}), numel(sequences{2})];
    accepted = [0, 0];
    hits = struct('azimuthDeg', {nan(totals(1), 1), nan(totals(2), 1)}, ...
        'clockSeconds', {nan(totals(1), 1), nan(totals(2), 1)}, ...
        'predicted', {strings(totals(1), 1), strings(totals(2), 1)});
    detectors = [initializeKickDetector, initializeKickDetector];
    armAzimuth = [NaN, NaN];
    localTimer = tic;
    turn = 1 + double(totals(1) == 0);
    announceDrumTarget(cfg.IMU.HandNames(turn), sequences{turn}(1), 1, ...
        totals(turn), phase);
    while any(accepted < totals)
        samples = cell(1, 2);
        for sensorIndex = 1:2
            samples{sensorIndex} = readMPU6050Sample(sensors(sensorIndex));
        end
        clockNow = toc(sessionClock);
        localTime = toc(localTimer);
        for sensorIndex = 1:2
            accelG = applyMPU6050AccelCalibration( ...
                samples{sensorIndex}.accelG, calibration.accel{sensorIndex});
            gyroRad = deg2rad(samples{sensorIndex}.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));
            modeBefore = detectors(sensorIndex).mode;
            azimuthBefore = estimators{sensorIndex}.AzimuthDeg();
            [detectors(sensorIndex), hit] = updateKickDetector( ...
                detectors(sensorIndex), accelG * 9.80665, gyroRad, ...
                calibration.directions(sensorIndex), localTime, ...
                sampleRateHz, cfg.Kick);
            if detectors(sensorIndex).mode == 2 && modeBefore ~= 2
                armAzimuth(sensorIndex) = azimuthBefore;
            end
            if hit && sensorIndex == turn && ...
                    accepted(sensorIndex) < totals(sensorIndex)
                accepted(sensorIndex) = accepted(sensorIndex) + 1;
                row = accepted(sensorIndex);
                azimuthForHit = azimuthBefore;
                if cfg.Regions.AzimuthFeatureTime == "arm" && ...
                        isfinite(armAzimuth(sensorIndex))
                    azimuthForHit = armAzimuth(sensorIndex);
                end
                hits(sensorIndex).azimuthDeg(row) = azimuthForHit;
                hits(sensorIndex).clockSeconds(row) = clockNow;
                if ~isempty(tracker)
                    hits(sensorIndex).predicted(row) = tracker.classify( ...
                        sensorIndex, azimuthForHit, true);
                end
                other = 3 - sensorIndex;
                if accepted(other) < totals(other)
                    turn = other;
                elseif accepted(sensorIndex) < totals(sensorIndex)
                    turn = sensorIndex;
                end
                if any(accepted < totals)
                    announceDrumTarget(cfg.IMU.HandNames(turn), ...
                        sequences{turn}(accepted(turn) + 1), ...
                        accepted(turn) + 1, totals(turn), phase);
                end
            end
            estimators{sensorIndex}.update(accelG, gyroRad, clockNow);
        end
    end
end

function printAzimuthSummary(model, cfg)
    for hand = 1:2
        h = model.hands(hand);
        positions = strings(1, numel(model.targetNames));
        for k = 1:numel(model.targetNames)
            positions(k) = sprintf('%s %+.0f', model.targetNames(k), ...
                h.centersDeg(k));
        end
        fprintf(['  %s hand: %s (deg) | min spacing %.0f deg | drift ' ...
            '%+.2f deg/s | self-accuracy %.0f%% | %s\n'], ...
            cfg.IMU.HandNames(hand), strjoin(positions, ', '), ...
            h.minSeparationDeg, h.driftDegPerSec, 100 * h.selfAccuracy, ...
            ternary(h.ok, "OK", "NOT OK: " + h.reason));
    end
end

function value = ternary(condition, whenTrue, whenFalse)
    if condition
        value = whenTrue;
    else
        value = whenFalse;
    end
end

function shutdownDualKick(logger, players, fig, cfg, metadataFolder)
    try
        logger.close();
    catch saveError
        warning('MyoDrum:FinalLogSaveFailed', ...
            'Could not flush the final sensor buffer: %s', saveError.message);
    end
    for playerIndex = 1:numel(players)
        try
            stop(players{playerIndex});
        catch
        end
    end
    if isgraphics(fig)
        delete(fig);
    end
    fprintf('\nTwo-IMU session stopped.\n');
    fprintf('Logged %d paired samples. Hits: %s=%d, %s=%d.\n', ...
        logger.LoggedSamples, cfg.IMU.HandNames(1), logger.KickCount(1), ...
        cfg.IMU.HandNames(2), logger.KickCount(2));
    fprintf('Raw data:\n  %s\n', logger.RawFile);
    fprintf('Metadata and calibration:\n  %s\n', metadataFolder);
    durations = logger.LoopDurationHistory;
    durations = durations(isfinite(durations));
    if ~isempty(durations)
        fprintf(['Loop duration median/p95/p99/max: ' ...
            '%.3f / %.3f / %.3f / %.3f s.\n'], ...
            median(durations), percentileValue(durations, 95), ...
            percentileValue(durations, 99), max(durations));
    end
    fprintf('Invalid frames: %s=%d, %s=%d.\n', ...
        cfg.IMU.HandNames(1), logger.InvalidFrameCount(1), ...
        cfg.IMU.HandNames(2), logger.InvalidFrameCount(2));
end

function banks = playDrumLabel(banks, label)
    fieldName = matlab.lang.makeValidName(label);
    bank = banks.(fieldName);
    play(bank.players{bank.nextVoice});
    bank.nextVoice = mod(bank.nextVoice, numel(bank.players)) + 1;
    banks.(fieldName) = bank;
end

function players = flattenPlayerBanks(banks, targetNames)
    players = cell(0, 1);
    for targetIndex = 1:numel(targetNames)
        fieldName = matlab.lang.makeValidName(targetNames(targetIndex));
        players = [players; banks.(fieldName).players(:)]; %#ok<AGROW>
    end
end

function fig = createControlFigure(mode)
    fig = figure('Name', sprintf('MyoDrum %s control', mode), ...
        'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
        'Color', 'w', 'Position', [100, 100, 420, 110], ...
        'KeyPressFcn', @keyPressed, 'CloseRequestFcn', @closeRequested);
    uicontrol(fig, 'Style', 'text', ...
        'String', ['Playing: Q stops and saves. R recenters azimuth. ' ...
            'Gravity-zone drum selection is unaffected.'], ...
        'Units', 'normalized', 'Position', [0.05, 0.25, 0.9, 0.5], ...
        'BackgroundColor', 'w', 'FontSize', 11);
    setappdata(fig, 'MyoDrumRunning', true);
    setappdata(fig, 'MyoDrumRecenterRequested', false);
    drawnow;
end

function keyPressed(source, event)
    if strcmpi(event.Key, 'q')
        setappdata(source, 'MyoDrumRunning', false);
    elseif strcmpi(event.Key, 'r')
        setappdata(source, 'MyoDrumRecenterRequested', true);
    end
end

function closeRequested(source, ~)
    setappdata(source, 'MyoDrumRunning', false);
end

function value = percentileValue(values, percentage)
    values = sort(values(:));
    position = 1 + (numel(values) - 1) * percentage / 100;
    lowerIndex = floor(position);
    upperIndex = ceil(position);
    if lowerIndex == upperIndex
        value = values(lowerIndex);
    else
        fraction = position - lowerIndex;
        value = values(lowerIndex) * (1 - fraction) + ...
            values(upperIndex) * fraction;
    end
end
