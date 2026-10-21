function mainIMUEMGStream(liveMode, classifierMode, sensorProfile)
% MAINIMUEMGSTREAM Synchronized 200 Hz IMU drum selection + EMG confirmation.

    arguments
        liveMode (1,1) string = "prompt"
        classifierMode (1,1) string = "rule-based"
        sensorProfile (1,1) string = "four-channel"
    end

    clc;
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    addpath(fullfile(root, 'classifiers', 'machineLearning'));
    addpath(fullfile(root, 'audio'));
    cfg = myoDrumConfig(root, sensorProfile);
    activeHands = cfg.IMU.ActiveHandIndices;
    activeChannels = cfg.EMG.ActiveChannelMask(:);
    classifierMode = lower(strtrim(classifierMode));
    if ~ismember(classifierMode, ...
            ["rule-based", "machine-learning", "hybrid"])
        error('MyoDrum:UnsupportedLiveClassifier', ...
            ['Live classifier must be rule-based, machine-learning ' ...
             'or hybrid.']);
    end
    if classifierMode ~= "rule-based"
        mlClassifier = initializeLiveMLClassifier(string(root), cfg, ...
            ClassifierMode=classifierMode);
    else
        mlClassifier = struct;
    end
    liveMode = selectLiveMode(liveMode);
    recordLiveData = liveMode == "recording";
    [accelCalibration, directions, regionModel] = loadCalibrations(cfg);

    sessionId = "session_" + string(datetime('now', ...
        'Format', 'yyyyMMdd_HHmmss'));
    startedAt = string(datetime('now', 'TimeZone', 'local'));
    rawFolder = fullfile(cfg.Paths.RawData, sessionId);
    metadataFolder = fullfile(cfg.Paths.Metadata, 'sessions', sessionId);
    mkdir(rawFolder); mkdir(metadataFolder);
    rawFile = fullfile(rawFolder, 'imuEmgStream.mat');
    metadataFile = fullfile(metadataFolder, 'sessionMetadata.mat');
    metadata.sessionId = sessionId;
    metadata.startedAt = startedAt;
    metadata.mode = "imu-emg-stream";
    metadata.classifierMode = classifierMode;
    metadata.sensorProfile = cfg.Runtime.SensorProfile;
    metadata.config = cfg;
    if recordLiveData
        metadata.rawDataFile = string(rawFile);
    else
        metadata.rawDataFile = "";
    end
    metadata.liveMode = liveMode;
    metadata.liveRecordingEnabled = recordLiveData;
    metadata.completedNormally = false;
    metadata.timestampSource = ...
        "Arduino micros() shared by both IMUs and all EMG channels";
    save(metadataFile, 'metadata');

    fprintf('MyoDrum: synchronized IMU + EMG stream\n');
    fprintf('Session: %s\n', sessionId);
    fprintf('Classifier: %s\n', upper(classifierMode));
    if recordLiveData
        fprintf('Live mode: RESEARCH RECORDING (continuous data retained).\n');
    else
        fprintf('Live mode: PERFORMANCE (no continuous live data recording).\n');
    end
    fprintf('Firmware target: %.0f complete synchronized frames/s.\n', ...
        cfg.EMG.CombinedStreamRateHz);
    if cfg.Runtime.SensorProfile == "left-three-emg"
        fprintf('Upload firmware/MyoDrumLeft3EMGStream first.\n');
    else
        fprintf('Upload firmware/MyoDrumIMUEMGStream first.\n');
    end
    reader = IMUEMGStreamReader(cfg.Hardware.COMPort, ...
        cfg.EMG.CombinedStreamBaud);
    cleanupReader = onCleanup(@()delete(reader));

    fprintf('\nREST/BIAS CALIBRATION\n');
    if numel(activeHands) == 1
        input(['Keep the LEFT stick and left-arm muscles relaxed, then ' ...
            'press Enter...'], 's');
    else
        input('Keep both sticks and all muscles relaxed, then press Enter...', 's');
    end
    reader.start();
    pause(cfg.IMU.StreamStartupSettleSeconds);
    discardPendingFrames(reader, cfg);
    restFrames = collectForSeconds(reader, max(3, cfg.EMG.RestSeconds), cfg);
    ensureHealthyFrames(restFrames, cfg);
    gyroBiasDps = zeros(2, 3);
    for hand = activeHands
        gyroBiasDps(hand, :) = median( ...
            restFrames.gyroDps(:, :, hand), 1);
    end
    restVoltage = adcToVolts(restFrames.emgAdc, cfg);

    activeFrames = emptyFrames;
    activeSampleHands = strings(0,1);
    for hand = activeHands
        fprintf('\n%s EMG GAIN CALIBRATION\n', cfg.IMU.HandNames(hand));
        input(sprintf(['Keep the other arm relaxed. Press Enter to START, ' ...
            'then immediately use the %s hand to play comfortable KICK, ' ...
            'SNARE and CRASH gestures for %.0f seconds...'], ...
            cfg.IMU.HandNames(hand), ...
            cfg.EMG.ActiveCalibrationSeconds), 's');
        discardPendingFrames(reader, cfg);
        fprintf('RECORDING %s EMG NOW...\n', cfg.IMU.HandNames(hand));
        trial = collectForSeconds(reader, ...
            cfg.EMG.ActiveCalibrationSeconds, cfg);
        activeFrames = appendFrames(activeFrames, trial);
        activeSampleHands = [activeSampleHands; repmat( ...
            cfg.IMU.HandNames(hand), numel(trial.time_s), 1)]; %#ok<AGROW>
    end
    activeVoltage = adcToVolts(activeFrames.emgAdc, cfg);
    profile = buildEMGConfirmationProfile(restVoltage, activeVoltage, ...
        cfg.EMG.Hands, MinimumActiveSpanVolts= ...
        cfg.EMG.MinimumActiveSpanVolts, ...
        ActiveSampleHands=activeSampleHands, ...
        ActiveChannelMask=cfg.EMG.ActiveChannelMask, ...
        RailVolts=cfg.EMG.Quality.RailVolts, ...
        MaximumRailFraction=cfg.EMG.Quality.MaximumRailFraction, ...
        MaximumRestSigmaVolts=cfg.EMG.Quality.RestSigmaWarnVolts, ...
        MinimumSignalToNoiseRatio= ...
        cfg.EMG.Quality.MinimumSignalToNoiseRatio, ...
        NoiseFloorVolts=cfg.EMG.Quality.NoiseFloorVolts);
    emgCalibration.restFrames = restFrames;
    emgCalibration.activeFrames = activeFrames;
    emgCalibration.activeSampleHands = activeSampleHands;
    emgCalibration.profile = profile;
    save(fullfile(rawFolder, 'emgConfirmationCalibration.mat'), ...
        'emgCalibration', '-v7.3');
    printEMGProfile(profile, cfg);
    if ~all(profile.validChannel(activeChannels))
        invalid = activeChannels & ~profile.validChannel(:);
        reasons = profile.qualityReason(:);
        details = cfg.EMG.ChannelNames(invalid) + " (" + ...
            reasons(invalid) + ")";
        metadata.emgProfile = profile;
        metadata.emgSafetyCheckPassed = false;
        save(metadataFile, 'metadata');
        error('MyoDrum:UnsafeEMGCalibration', ...
            ['EMG hardware check failed: %s. Audio was NOT enabled. ' ...
             'Reconnect the envelope outputs, check the common ground, ' ...
             'reduce any saturating gain, and retry.'], ...
            strjoin(details, ', '));
    end
    metadata.emgProfile = profile;
    metadata.emgSafetyCheckPassed = true;
    save(metadataFile, 'metadata');
    fprintf('EMG safety check passed. Audio can now be enabled.\n');

    estimators = cell(1, 2);
    for hand = activeHands
        estimators{hand} = GravityAzimuthEstimator( ...
            cfg.Orientation.EstimatorOverrides);
        estimators{hand}.setStrokeAxis(directions(hand).axis);
    end
    for row = 1:numel(restFrames.time_s)
        for hand = activeHands
            accel = applyMPU6050AccelCalibration( ...
                reshape(restFrames.accelG(row, :, hand), 1, 3), ...
                accelCalibration{hand});
            gyro = deg2rad(reshape(restFrames.gyroDps(row, :, hand), 1, 3) ...
                - gyroBiasDps(hand, :));
            estimators{hand}.update(accel, gyro, restFrames.time_s(row));
        end
    end
    detectors = [initializeKickDetector, initializeKickDetector];
    banks = createDrumPlayerBanks(cfg.Paths.Audio, cfg.Targets.Names, ...
        cfg.Targets.AudioFiles, cfg.Kick.AudioVoices);
    showDashboard = cfg.Visualization.Enabled && recordLiveData;
    if showDashboard
        dashboard = createLiveSensorDashboard(cfg);
        fig = dashboard.Figure;
    else
        dashboard = struct;
        fig = createControlFigure;
    end
    cleanupFigure = onCleanup(@()closeFigure(fig));

    if recordLiveData
        liveLog = initializeIMUEMGLiveLog(cfg.Logging.BufferSamples);
    else
        liveLog = struct;
    end
    activeRegion = strings(1, 2);
    armedRegion = strings(1, 2);
    armGravity = nan(2, 3);
    peakGyro = zeros(2, 3);
    peakMagnitude = zeros(1, 2);
    fusion = initializeFusionClassifier;
    lastRailTime_s = -inf(1, numel(cfg.EMG.ChannelNames));
    runtimeFaultReported = false(1, numel(cfg.EMG.ChannelNames));
    fusionEventCells = cell(0, 1);
    candidateCount = zeros(1, 2);
    acceptedCount = zeros(1, 2);
    rejectedCount = zeros(1, 2);
    lastCheckpointTime = -inf;
    lastDashboardTime = -inf;
    displayLabels = strings(1, 2);
    displayScores = zeros(1, 2);
    discardPendingFrames(reader, cfg);
    runtimeClock = tic;
    hostTimeOffset_s = NaN;
    lastFrameTime_s = NaN;
    if classifierMode == "machine-learning"
        fprintf(['\nREADY. ML selects KICK, SNARE, CRASH or NO_HIT; the ' ...
            'configured EMG gate must confirm sound. Press Q or close ' ...
            'the window to stop.\n\n']);
    elseif classifierMode == "hybrid"
        fprintf(['\nREADY. Rule-based and ML drum labels must agree, and ' ...
            'the configured EMG gate must confirm sound. Press Q or ' ...
            'close the window to stop.\n\n']);
    else
        fprintf(['\nREADY. IMU selects the drum; the configured EMG gate ' ...
            'must confirm each hit. Press Q or close the window to stop.\n\n']);
    end

    while isgraphics(fig) && getappdata(fig, 'MyoDrumRunning')
        frames = reader.readAvailableFrames();
        if isempty(frames.sequence)
            pause(0.001); drawnow limitrate; continue;
        end
        batchHostTime_s = toc(runtimeClock);
        if isnan(hostTimeOffset_s)
            % The newest frame in the first live batch defines zero host
            % backlog. Later values expose serial/processing delay directly.
            hostTimeOffset_s = batchHostTime_s - frames.time_s(end);
        end
        volts = adcToVolts(frames.emgAdc, cfg);
        for row = 1:numel(frames.sequence)
            normalizedEMG = max(0, (volts(row, :) - ...
                profile.baseline_V) ./ profile.activeSpan_V);
            railed = volts(row, :) >= cfg.EMG.Quality.RailVolts;
            lastRailTime_s(railed) = frames.time_s(row);
            runtimeValid = profile.validChannel & activeChannels.' & ...
                frames.time_s(row) - lastRailTime_s >= ...
                cfg.EMG.Quality.RuntimeRailHoldSeconds;
            normalizedEMG(~runtimeValid) = 0;
            newlyFaulted = activeChannels.' & ~runtimeValid & ...
                ~runtimeFaultReported;
            if any(newlyFaulted)
                warning('MyoDrum:EMGRuntimeRail', ...
                    ['EMG channel(s) %s railed at runtime and were ' ...
                     'temporarily removed from hit confirmation.'], ...
                    strjoin(cfg.EMG.ChannelNames(newlyFaulted), ', '));
                runtimeFaultReported(newlyFaulted) = true;
            end
            rawAccel = reshape(frames.accelG(row, :, :), 1, 3, 2);
            calibratedAccel = zeros(1, 3, 2);
            correctedGyro = zeros(1, 3, 2);
            ypr = zeros(1, 3, 2);
            candidateHit = false(1, 2);
            candidateLabels = strings(1, 2);
            for hand = activeHands
                if ~frames.frameValid(row, hand), continue; end
                accel = applyMPU6050AccelCalibration( ...
                    reshape(frames.accelG(row, :, hand), 1, 3), ...
                    accelCalibration{hand});
                gyro = deg2rad(reshape(frames.gyroDps(row, :, hand), 1, 3) ...
                    - gyroBiasDps(hand, :));
                gravityBefore = estimators{hand}.GravityBody;
                activeRegion(hand) = classifyGravityDrum(regionModel, hand, ...
                    gravityBefore, MaximumDistance= ...
                    cfg.Regions.MaximumNormalizedDistance, ...
                    AmbiguityMargin=cfg.Regions.AmbiguityMargin, ...
                    CurrentLabel=activeRegion(hand), ...
                    ExitDistance=cfg.Regions.GravityZoneExitDistance);
                modeBefore = detectors(hand).mode;
                [detectors(hand), hit] = updateKickDetector(detectors(hand), ...
                    accel * 9.80665, gyro, directions(hand), ...
                    frames.time_s(row), cfg.EMG.CombinedStreamRateHz, cfg.Kick);
                magnitude = norm(gyro);
                signedRate = directions(hand).sign * gyro(directions(hand).axis);
                if detectors(hand).mode == 2 && modeBefore ~= 2
                    armGravity(hand, :) = gravityBefore;
                    armedRegion(hand) = activeRegion(hand);
                    peakGyro(hand, :) = gyro;
                    peakMagnitude(hand) = magnitude;
                elseif modeBefore == 2 && signedRate > 0 && ...
                        magnitude > peakMagnitude(hand)
                    peakGyro(hand, :) = gyro;
                    peakMagnitude(hand) = magnitude;
                end
                candidateHit(hand) = hit;
                if hit
                    candidateLabels(hand) = classifyGravityDrum( ...
                        regionModel, hand, ...
                        armGravity(hand, :), MaximumDistance= ...
                        cfg.Regions.MaximumNormalizedDistance, ...
                        AmbiguityMargin=cfg.Regions.AmbiguityMargin, ...
                        CurrentLabel=armedRegion(hand), ...
                        ExitDistance=cfg.Regions.GravityZoneExitDistance, ...
                        GyroPeakVector=peakGyro(hand, :), ...
                        MaximumGyroAngleDeg= ...
                        cfg.Regions.GravityZoneMaximumGyroAngleDeg);
                end
                estimators{hand}.update(accel, gyro, frames.time_s(row));
                calibratedAccel(1, :, hand) = accel;
                correctedGyro(1, :, hand) = gyro;
                ypr(1, :, hand) = estimators{hand}.Features();
            end
            if classifierMode ~= "rule-based"
                ruleCandidateLabels = candidateLabels;
                [mlClassifier, decision, events] = ...
                    updateLiveMLClassifier(mlClassifier, ...
                    frames.time_s(row), calibratedAccel, correctedGyro, ...
                    normalizedEMG, frames.frameValid(row,:), ...
                    runtimeValid, candidateHit, cfg, ruleCandidateLabels);
                candidateLabels = decision.labels;
            else
                [fusion, decision, events] = updateFusionClassifier( ...
                    fusion, frames.time_s(row), candidateHit, ...
                    candidateLabels, normalizedEMG, cfg.EMG.Hands, ...
                    runtimeValid(:), cfg.Fusion);
            end
            displayScores = decision.emgScore;
            for hand = activeHands
                if candidateLabels(hand) ~= ""
                    displayLabels(hand) = candidateLabels(hand);
                end
            end
            targetIndex = zeros(1, 2, 'uint8');
            candidateTargetIndex = zeros(1, 2, 'uint8');
            for hand = activeHands
                if candidateLabels(hand) ~= ""
                    match = find(cfg.Targets.Names == ...
                        candidateLabels(hand), 1);
                    if ~isempty(match)
                        candidateTargetIndex(hand) = uint8(match);
                    end
                end
                if decision.accepted(hand) && decision.labels(hand) ~= ""
                    match = find(cfg.Targets.Names == ...
                        decision.labels(hand), 1);
                    if ~isempty(match)
                        targetIndex(hand) = uint8(match);
                    end
                end
            end
            for eventIndex = 1:numel(events)
                if recordLiveData
                    fusionEventCells{end+1,1} = events(eventIndex); %#ok<AGROW>
                end
            end
            candidateCount = candidateCount + double(candidateHit);
            acceptedCount = acceptedCount + double(decision.accepted);
            rejectedCount = rejectedCount + double(decision.rejected);
            % Play once per accepted hand. Do not collapse equal labels: a
            % simultaneous KICK from both sticks must use two audio voices.
            for hand = activeHands
                if decision.accepted(hand) && decision.labels(hand) ~= ""
                    banks = playLabel(banks, decision.labels(hand));
                end
            end
            % Console rendering is deliberately after audio so diagnostic
            % feedback can never delay an accepted hit's sound trigger.
            if cfg.Runtime.PrintHitEvents
                for eventIndex = 1:numel(events)
                    if classifierMode == "hybrid"
                        fprintf(['%s RULE %s + ML %s | %s | ML %.2f | ' ...
                            'EMG %.2f | latency %.0f ms\n'], ...
                            cfg.IMU.HandNames(events(eventIndex).handIndex), ...
                            events(eventIndex).ruleLabel, ...
                            events(eventIndex).mlLabel, ...
                            events(eventIndex).reason, ...
                            events(eventIndex).mlConfidence, ...
                            events(eventIndex).peakEMGScore, ...
                            1000 * events(eventIndex).decisionLatency_s);
                    elseif classifierMode == "machine-learning"
                        fprintf(['%s %s | %s | ML %.2f | EMG %.2f | ' ...
                            'latency %.0f ms\n'], ...
                            cfg.IMU.HandNames(events(eventIndex).handIndex), ...
                            events(eventIndex).orientationLabel, ...
                            events(eventIndex).reason, ...
                            events(eventIndex).mlConfidence, ...
                            events(eventIndex).peakEMGScore, ...
                            1000 * events(eventIndex).decisionLatency_s);
                    else
                        fprintf('%s %s | %s | EMG %.2f | latency %.0f ms\n', ...
                            cfg.IMU.HandNames(events(eventIndex).handIndex), ...
                            events(eventIndex).orientationLabel, ...
                            events(eventIndex).reason, ...
                            events(eventIndex).peakEMGScore, ...
                            1000 * events(eventIndex).decisionLatency_s);
                    end
                end
            end
            if recordLiveData
                hostProcessingLag_s = max(0, toc(runtimeClock) - ...
                    frames.time_s(row) - hostTimeOffset_s);
                liveLog = appendIMUEMGLiveLog(liveLog, frames, row, ...
                    rawAccel, calibratedAccel, correctedGyro, ypr, ...
                    volts(row, :), normalizedEMG, candidateHit, decision, ...
                    candidateTargetIndex, targetIndex, runtimeValid, ...
                    hostProcessingLag_s);
            end
        end
        currentTime_s = frames.time_s(end);
        lastFrameTime_s = currentTime_s;
        if showDashboard && recordLiveData && liveLog.sampleCount > 0 && ...
                currentTime_s - lastDashboardTime >= ...
                1 / cfg.Visualization.RefreshRateHz
            dashboardRows = max(cfg.Visualization.MaximumPoints, ...
                ceil(1.25 * cfg.Visualization.WindowSeconds * ...
                cfg.EMG.CombinedStreamRateHz));
            firstDashboardRow = max(1, ...
                liveLog.sampleCount-dashboardRows+1);
            dashboardLog = snapshotIMUEMGLiveLog(liveLog, ...
                firstDashboardRow:liveLog.sampleCount);
            updateLiveSensorDashboard(dashboard, dashboardLog, cfg, ...
                displayLabels, displayScores);
            lastDashboardTime = currentTime_s;
        elseif ~showDashboard
            drawnow limitrate;
        end
        if recordLiveData && isfinite(cfg.Logging.StreamCheckpointSeconds) && ...
                liveLog.sampleCount > 0 && ...
                currentTime_s-lastCheckpointTime >= ...
                cfg.Logging.StreamCheckpointSeconds
            checkpoint = snapshotIMUEMGLiveLog(liveLog);
            checkpoint.fusionEvents = eventsToTable(fusionEventCells);
            save(rawFile, '-struct', 'checkpoint', '-v7.3');
            metadata.loggedSamples = liveLog.sampleCount;
            metadata.lastCheckpointTime_s = currentTime_s;
            metadata.emgProfile = profile;
            metadata.completedNormally = false;
            save(metadataFile, 'metadata');
            lastCheckpointTime = currentTime_s;
        end
    end
    if isfinite(lastFrameTime_s) && classifierMode == "rule-based"
        [fusion, ~, finalEvents] = updateFusionClassifier(fusion, ...
            lastFrameTime_s+ ...
            cfg.Fusion.EMGConfirmationTimeoutSeconds+eps, ...
            false(1,2), strings(1,2), zeros(1,4), cfg.EMG.Hands, ...
            profile.validChannel(:) & activeChannels, cfg.Fusion); %#ok<ASGLU>
        for eventIndex = 1:numel(finalEvents)
            hand = double(finalEvents(eventIndex).handIndex);
            if finalEvents(eventIndex).accepted
                acceptedCount(hand) = acceptedCount(hand) + 1;
            else
                rejectedCount(hand) = rejectedCount(hand) + 1;
            end
            if recordLiveData
                fusionEventCells{end+1,1} = finalEvents(eventIndex); %#ok<AGROW>
            end
        end
    end
    reader.stop();
    statistics = reader.getStatistics();
    if recordLiveData
        log = snapshotIMUEMGLiveLog(liveLog);
        log.fusionEvents = eventsToTable(fusionEventCells);
        metadata = createMetadata(sessionId, startedAt, cfg, rawFile, ...
            profile, gyroBiasDps, statistics, log);
        metadata.classifierMode = classifierMode;
        metadata.liveMode = liveMode;
        metadata.liveRecordingEnabled = true;
        save(rawFile, '-struct', 'log', '-v7.3');
    else
        metadata.endedAt = string(datetime('now','TimeZone','local'));
        metadata.emgProfile = profile;
        metadata.gyroBiasDps = gyroBiasDps;
        metadata.streamStatistics = statistics;
        metadata.loggedSamples = 0;
        metadata.candidateHitCount = candidateCount;
        metadata.confirmedHitCount = acceptedCount;
        metadata.rejectedHitCount = rejectedCount;
        metadata.completedNormally = true;
        metadata.classifierMode = classifierMode;
    end
    save(metadataFile, 'metadata');
    if recordLiveData
        fprintf(['\nSaved %d synchronized frames at %.2f Hz; dropped %d, ' ...
            'CRC failures %d.\nHost processing lag median/p95/max: ' ...
            '%.1f / %.1f / %.1f ms.\nRaw data:\n  %s\n'], ...
            numel(log.time_s), statistics.measuredRateHz, ...
            statistics.droppedFrames, statistics.checksumFailures, ...
            1000 * median(log.hostProcessingLag_s, 'omitnan'), ...
            1000 * prctile(log.hostProcessingLag_s, 95), ...
            1000 * max(log.hostProcessingLag_s, [], 'omitnan'), rawFile);
    else
        fprintf(['\nPerformance session stopped. No continuous live sensor ' ...
            'recording was created. Stream %.2f Hz; dropped %d, CRC ' ...
            'failures %d.\nSession metadata:\n  %s\n'], ...
            statistics.measuredRateHz, statistics.droppedFrames, ...
            statistics.checksumFailures, metadataFile);
    end
    clear cleanupFigure cleanupReader;
end

function mode = selectLiveMode(mode)
    mode = lower(strtrim(mode));
    if mode == "prompt"
        fprintf('\nSelect the live session mode:\n');
        fprintf('  1. Performance (no continuous recording) [recommended]\n');
        fprintf('  2. Research recording (save all IMU and EMG frames)\n');
        choice = input('Selection: ');
        if ~isscalar(choice) || ~isnumeric(choice) || ~isfinite(choice) || ...
                choice ~= fix(choice) || choice < 1 || choice > 2
            error('MyoDrum:InvalidLiveModeSelection', ...
                'Enter either 1 for performance or 2 for recording.');
        end
        modes = ["performance", "recording"];
        mode = modes(choice);
    end
    switch mode
        case {"performance", "play", "no-recording"}
            mode = "performance";
        case {"recording", "research", "record"}
            mode = "recording";
        otherwise
            error('MyoDrum:UnknownLiveMode', ...
                'Unknown live session mode "%s".', mode);
    end
end

function result = eventsToTable(eventCells)
    if isempty(eventCells)
        result = table;
    else
        result = struct2table([eventCells{:}]);
    end
end

function [accel, directions, model] = loadCalibrations(cfg)
    folder = fullfile(cfg.Paths.Metadata, 'calibration');
    accel = cell(1, 2);
    for hand = cfg.IMU.ActiveHandIndices
        token = erase(cfg.IMU.Addresses(hand), '0x');
        file = fullfile(folder, "mpu6050_0x" + token + ...
            "_accelCalibration.mat");
        requireFile(file);
        saved = load(file, 'calibration'); accel{hand} = saved.calibration;
    end
    directionFile = fullfile(folder, 'twoIMUKickDirectionCalibration.mat');
    modelFile = fullfile(folder, 'threeDrumGravityRegionModel.mat');
    requireFile(directionFile); requireFile(modelFile);
    saved = load(directionFile, 'directions'); directions = saved.directions;
    saved = load(modelFile, 'model'); model = saved.model;
end

function requireFile(file)
    if ~isfile(file)
        error('MyoDrum:MissingStreamCalibration', ...
            'Required calibration is missing: %s', file);
    end
end

function frames = collectForSeconds(reader, seconds, cfg)
    frames = emptyFrames;
    startTime = [];
    phaseTimer = tic;
    lastFrameTimer = tic;
    while isempty(startTime) || isempty(frames.time_s) || ...
            frames.time_s(end) - startTime < seconds
        available = reader.readAvailableFrames();
        if isempty(available.sequence)
            if toc(lastFrameTimer) > cfg.EMG.StreamTimeoutSeconds
                error('MyoDrum:CombinedStreamTimeout', ...
                    ['No synchronized sensor frames arrived for %.1f ' ...
                     'seconds. Check that the combined firmware is running, ' ...
                     'then reconnect the Mega and retry.'], ...
                    cfg.EMG.StreamTimeoutSeconds);
            end
            pause(0.001);
            continue;
        end
        lastFrameTimer = tic;
        if isempty(startTime), startTime = available.time_s(1); end
        frames = appendFrames(frames, available);
        if toc(phaseTimer) > seconds + cfg.EMG.StreamTimeoutSeconds
            error('MyoDrum:CombinedStreamTooSlow', ...
                ['The synchronized stream did not provide %.1f seconds of ' ...
                 'data within the allowed time.'], seconds);
        end
    end
    expected = floor(seconds * cfg.EMG.CombinedStreamRateHz * 0.8);
    if numel(frames.sequence) < expected
        warning('MyoDrum:SlowCombinedStream', ...
            'Only %d frames arrived; expected at least %d.', ...
            numel(frames.sequence), expected);
    end
end

function discardPendingFrames(reader, cfg)
    % The firmware streams while MATLAB waits at operator prompts. Read and
    % discard that queued history until the host catches the live edge. Using
    % reads instead of serial flush also releases a Mega blocked in Serial.write.
    timer = tic;
    while true
        pending = reader.readAvailableFrames();
        if isempty(pending.sequence)
            return;
        end
        if toc(timer) > cfg.EMG.StreamTimeoutSeconds
            error('MyoDrum:CombinedStreamBacklog', ...
                ['MATLAB could not catch up with the synchronized stream. ' ...
                 'Reconnect the Mega and retry.']);
        end
    end
end

function frames = emptyFrames
    frames.sequence = zeros(0,1,'uint16'); frames.timestamp_us=zeros(0,1,'uint32');
    frames.time_s=zeros(0,1); frames.accelG=zeros(0,3,2);
    frames.gyroDps=zeros(0,3,2); frames.temperatureC=zeros(0,2);
    frames.emgAdc=zeros(0,4,'uint16'); frames.status=zeros(0,1,'uint8');
    frames.frameValid=false(0,2);
end

function combined = appendFrames(combined, addition)
    for name = string(fieldnames(combined)).'
        combined.(name) = cat(1, combined.(name), addition.(name));
    end
end

function ensureHealthyFrames(frames, cfg)
    if numel(frames.sequence) < cfg.EMG.CombinedStreamRateHz
        error('MyoDrum:InsufficientCombinedFrames', ...
            'Too few valid synchronized frames arrived.');
    end
    validFraction = mean(frames.frameValid,1);
    failedFrames = sum(~frames.frameValid,1);
    disp(table(cfg.IMU.HandNames(:), cfg.IMU.Addresses(:), ...
        (100*validFraction(:)), failedFrames(:), ...
        'VariableNames', {'Hand','Address','ValidFrames_pct', ...
        'FailedFrames'}));
    active = cfg.IMU.ActiveHandIndices;
    if any(validFraction(active) < cfg.IMU.StreamMinimumValidFraction)
        failed = false(size(validFraction));
        failed(active) = validFraction(active) < ...
            cfg.IMU.StreamMinimumValidFraction;
        details = cfg.IMU.HandNames(failed) + " " + ...
            cfg.IMU.Addresses(failed) + "=" + ...
            compose('%.1f%%',100*validFraction(failed));
        error('MyoDrum:CombinedIMUReadFailures', ...
            ['MPU stream reliability is below %.0f%%: %s. Physical ' ...
             'connection can be present while I2C transactions fail; ' ...
             'check SDA/SCL length, pull-ups, power and address wiring.'], ...
            100*cfg.IMU.StreamMinimumValidFraction, strjoin(details, ', '));
    elseif any(validFraction(active) < cfg.IMU.StreamWarningValidFraction)
        warning('MyoDrum:IntermittentIMUReadFailures', ...
            ['One or more MPUs had intermittent startup read failures. ' ...
             'Live processing will discard those isolated frames.']);
    end
end

function volts = adcToVolts(adc, cfg)
    volts = double(adc) * cfg.EMG.ADCReferenceVolts / cfg.EMG.ADCMaxCount;
end

function printEMGProfile(profile, cfg)
    active = cfg.EMG.ActiveChannelMask(:);
    disp(table(cfg.EMG.ChannelNames(active), cfg.EMG.AnalogPins(active), ...
        profile.baseline_V(active)', profile.noiseSigma_V(active)', ...
        profile.activeReference_V(active)', profile.activeSpan_V(active)', ...
        100 * profile.restRailFraction(active)', ...
        100 * profile.activeRailFraction(active)', ...
        profile.signalToNoiseRatio(active)', profile.validChannel(active)', ...
        profile.qualityReason(active)', 'VariableNames', ...
        {'Channel','Pin','Rest_V','RestNoise_V','ActiveReference_V', ...
         'Span_V','RestRail_pct','ActiveRail_pct','SignalToNoise', ...
         'Valid','Reason'}));
end

function fig = createControlFigure
    fig = figure('Name','MyoDrum synchronized IMU+EMG', ...
        'NumberTitle','off','Visible','on','KeyPressFcn',@keyPressed, ...
        'CloseRequestFcn',@closeRequested);
    setappdata(fig,'MyoDrumRunning',true);
end
function keyPressed(fig,event)
    if strcmpi(event.Key,'q'), setappdata(fig,'MyoDrumRunning',false); end
end
function closeRequested(fig,~), setappdata(fig,'MyoDrumRunning',false); end
function closeFigure(fig), if isgraphics(fig), delete(fig); end, end

function banks = playLabel(banks, label)
    field = matlab.lang.makeValidName(label);
    voice = banks.(field).nextVoice;
    stop(banks.(field).players{voice}); play(banks.(field).players{voice});
    banks.(field).nextVoice = mod(voice, numel(banks.(field).players)) + 1;
end

function metadata = createMetadata(id,startedAt,cfg,file,profile,bias,stats,log)
    metadata.sessionId=id; metadata.startedAt=startedAt;
    metadata.endedAt=string(datetime('now','TimeZone','local'));
    metadata.mode="imu-emg-stream"; metadata.config=cfg;
    metadata.rawDataFile=string(file); metadata.emgProfile=profile;
    metadata.gyroBiasDps=bias; metadata.streamStatistics=stats;
    metadata.loggedSamples=numel(log.time_s);
    metadata.candidateHitCount=sum(log.imuCandidateHit,1);
    metadata.confirmedHitCount=sum(log.fusionAcceptedHit,1);
    metadata.rejectedHitCount=sum(log.fusionRejectedHit,1);
    metadata.hostProcessingLagMedian_s=median( ...
        log.hostProcessingLag_s,'omitnan');
    metadata.hostProcessingLagP95_s=prctile( ...
        log.hostProcessingLag_s,95);
    metadata.hostProcessingLagMaximum_s=max( ...
        log.hostProcessingLag_s,[],'omitnan');
    metadata.fusionDecisionCodes=struct('none',uint8(0), ...
        'accepted',uint8(1),'emgTimeout',uint8(2), ...
        'uncertainOrientation',uint8(3),'duplicateGuard',uint8(4), ...
        'pendingAlreadyActive',uint8(5));
    metadata.timestampSource="Arduino micros() shared by both IMUs and all EMG channels";
    metadata.completedNormally=true;
end
