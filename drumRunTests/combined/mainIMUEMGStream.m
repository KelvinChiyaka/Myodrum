function mainIMUEMGStream
% MAINIMUEMGSTREAM Synchronized 200 Hz IMU drum selection + EMG confirmation.

    clc;
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    addpath(fullfile(root, 'audio'));
    cfg = myoDrumConfig(root);
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
    metadata.config = cfg;
    metadata.rawDataFile = string(rawFile);
    metadata.completedNormally = false;
    metadata.timestampSource = ...
        "Arduino micros() shared by both IMUs and all EMG channels";
    save(metadataFile, 'metadata');

    fprintf('MyoDrum: synchronized dual-IMU + EMG stream\n');
    fprintf('Session: %s\n', sessionId);
    fprintf(['Firmware target: %.0f complete synchronized frames/s.\n' ...
        'Upload firmware/MyoDrumIMUEMGStream first.\n'], ...
        cfg.EMG.CombinedStreamRateHz);
    reader = IMUEMGStreamReader(cfg.Hardware.COMPort, ...
        cfg.EMG.CombinedStreamBaud);
    cleanupReader = onCleanup(@()delete(reader));
    reader.start();

    fprintf('\nREST/BIAS CALIBRATION\n');
    input('Keep both sticks and all muscles relaxed, then press Enter...', 's');
    restFrames = collectForSeconds(reader, max(3, cfg.EMG.RestSeconds), cfg);
    ensureHealthyFrames(restFrames, cfg);
    gyroBiasDps = zeros(2, 3);
    for hand = 1:2
        gyroBiasDps(hand, :) = median( ...
            restFrames.gyroDps(:, :, hand), 1);
    end
    restVoltage = adcToVolts(restFrames.emgAdc, cfg);

    activeFrames = emptyFrames;
    for hand = 1:2
        fprintf('\n%s EMG GAIN CALIBRATION\n', cfg.IMU.HandNames(hand));
        input(sprintf(['Keep the other arm relaxed. With the %s hand, play ' ...
            'comfortable KICK, SNARE and CRASH gestures repeatedly, then ' ...
            'press Enter...'], cfg.IMU.HandNames(hand)), 's');
        trial = collectForSeconds(reader, ...
            cfg.EMG.ActiveCalibrationSeconds, cfg);
        activeFrames = appendFrames(activeFrames, trial);
    end
    activeVoltage = adcToVolts(activeFrames.emgAdc, cfg);
    profile = buildEMGConfirmationProfile(restVoltage, activeVoltage, ...
        cfg.EMG.Hands, MinimumActiveSpanVolts= ...
        cfg.EMG.MinimumActiveSpanVolts);
    printEMGProfile(profile, cfg);
    if ~all(profile.validChannel)
        bad = strjoin(cfg.EMG.ChannelNames(~profile.validChannel), ', ');
        error('MyoDrum:WeakEMGCalibration', ...
            ['These channels did not rise enough above rest: %s. Adjust ' ...
             'their individual amplifier gains/electrodes and retry.'], bad);
    end
    emgCalibration.restFrames = restFrames;
    emgCalibration.activeFrames = activeFrames;
    emgCalibration.profile = profile;
    save(fullfile(rawFolder, 'emgConfirmationCalibration.mat'), ...
        'emgCalibration', '-v7.3');

    estimators = cell(1, 2);
    for hand = 1:2
        estimators{hand} = GravityAzimuthEstimator( ...
            cfg.Orientation.EstimatorOverrides);
        estimators{hand}.setStrokeAxis(directions(hand).axis);
    end
    for row = 1:numel(restFrames.time_s)
        for hand = 1:2
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
    fig = createControlFigure;
    cleanupFigure = onCleanup(@()closeFigure(fig));

    log = emptyLog;
    activeRegion = strings(1, 2);
    armedRegion = strings(1, 2);
    armGravity = nan(2, 3);
    peakGyro = zeros(2, 3);
    peakMagnitude = zeros(1, 2);
    fusion = initializeFusionClassifier;
    fusionEventCells = cell(0, 1);
    lastCheckpointTime = -inf;
    fprintf(['\nREADY. IMU selects the drum; independently normalized EMG ' ...
        'confirms each hit. Press Q or close the window to stop.\n\n']);

    while isgraphics(fig) && getappdata(fig, 'MyoDrumRunning')
        frames = reader.readAvailableFrames();
        if isempty(frames.sequence)
            pause(0.001); drawnow limitrate; continue;
        end
        volts = adcToVolts(frames.emgAdc, cfg);
        for row = 1:numel(frames.sequence)
            normalizedEMG = max(0, (volts(row, :) - ...
                profile.baseline_V) ./ profile.activeSpan_V);
            rawAccel = reshape(frames.accelG(row, :, :), 1, 3, 2);
            calibratedAccel = zeros(1, 3, 2);
            correctedGyro = zeros(1, 3, 2);
            ypr = zeros(1, 3, 2);
            candidateHit = false(1, 2);
            candidateLabels = strings(1, 2);
            for hand = 1:2
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
            [fusion, decision, events] = updateFusionClassifier( ...
                fusion, frames.time_s(row), candidateHit, ...
                candidateLabels, normalizedEMG, cfg.EMG.Hands, ...
                profile.validChannel(:), cfg.Fusion);
            targetIndex = zeros(1, 2, 'uint8');
            candidateTargetIndex = zeros(1, 2, 'uint8');
            for hand = 1:2
                if candidateLabels(hand) ~= ""
                    candidateTargetIndex(hand) = uint8(find( ...
                        cfg.Targets.Names == candidateLabels(hand), 1));
                end
                if decision.accepted(hand) && decision.labels(hand) ~= ""
                    targetIndex(hand) = uint8(find( ...
                        cfg.Targets.Names == decision.labels(hand), 1));
                end
            end
            for eventIndex = 1:numel(events)
                fusionEventCells{end+1,1} = events(eventIndex); %#ok<AGROW>
                fprintf('%s %s | %s | EMG %.2f | latency %.0f ms\n', ...
                    cfg.IMU.HandNames(events(eventIndex).handIndex), ...
                    events(eventIndex).orientationLabel, ...
                    events(eventIndex).reason, ...
                    events(eventIndex).peakEMGScore, ...
                    1000 * events(eventIndex).decisionLatency_s);
            end
            acceptedLabels = decision.labels(decision.accepted & ...
                decision.labels ~= "");
            for label = unique(acceptedLabels, 'stable')
                banks = playLabel(banks, label);
            end
            log = appendLog(log, frames, row, rawAccel, calibratedAccel, ...
                correctedGyro, ypr, volts(row, :), profile, candidateHit, ...
                decision, candidateTargetIndex, targetIndex);
        end
        if ~isempty(log.time_s) && log.time_s(end)-lastCheckpointTime >= ...
                cfg.Logging.StreamCheckpointSeconds
            checkpoint = log;
            checkpoint.fusionEvents = eventsToTable(fusionEventCells);
            save(rawFile, '-struct', 'checkpoint', '-v7.3');
            metadata.loggedSamples = numel(log.time_s);
            metadata.lastCheckpointTime_s = log.time_s(end);
            metadata.emgProfile = profile;
            metadata.completedNormally = false;
            save(metadataFile, 'metadata');
            lastCheckpointTime = log.time_s(end);
        end
        drawnow limitrate;
    end
    if ~isempty(log.time_s)
        [fusion, ~, finalEvents] = updateFusionClassifier(fusion, ...
            log.time_s(end)+cfg.Fusion.EMGConfirmationTimeoutSeconds+eps, ...
            false(1,2), strings(1,2), zeros(1,4), cfg.EMG.Hands, ...
            profile.validChannel(:), cfg.Fusion); %#ok<ASGLU>
        for eventIndex = 1:numel(finalEvents)
            fusionEventCells{end+1,1} = finalEvents(eventIndex); %#ok<AGROW>
        end
    end
    reader.stop();
    statistics = reader.getStatistics();
    log.fusionEvents = eventsToTable(fusionEventCells);
    metadata = createMetadata(sessionId, startedAt, cfg, rawFile, profile, ...
        gyroBiasDps, statistics, log);
    save(rawFile, '-struct', 'log', '-v7.3');
    save(metadataFile, 'metadata');
    fprintf(['\nSaved %d synchronized frames at %.2f Hz; dropped %d, ' ...
        'CRC failures %d.\nRaw data:\n  %s\n'], ...
        numel(log.time_s), statistics.measuredRateHz, ...
        statistics.droppedFrames, statistics.checksumFailures, rawFile);
    clear cleanupFigure cleanupReader;
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
    for hand = 1:2
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
    while isempty(startTime) || isempty(frames.time_s) || ...
            frames.time_s(end) - startTime < seconds
        available = reader.readAvailableFrames();
        if isempty(available.sequence), pause(0.001); continue; end
        if isempty(startTime), startTime = available.time_s(1); end
        frames = appendFrames(frames, available);
    end
    expected = floor(seconds * cfg.EMG.CombinedStreamRateHz * 0.8);
    if numel(frames.sequence) < expected
        warning('MyoDrum:SlowCombinedStream', ...
            'Only %d frames arrived; expected at least %d.', ...
            numel(frames.sequence), expected);
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
    if any(mean(frames.frameValid,1) < 0.99)
        error('MyoDrum:CombinedIMUReadFailures', ...
            'At least one MPU6050 failed more than 1%% of stream reads.');
    end
end

function volts = adcToVolts(adc, cfg)
    volts = double(adc) * cfg.EMG.ADCReferenceVolts / cfg.EMG.ADCMaxCount;
end

function printEMGProfile(profile, cfg)
    disp(table(cfg.EMG.ChannelNames, cfg.EMG.AnalogPins, ...
        profile.baseline_V', profile.activeReference_V', ...
        profile.activeSpan_V', profile.validChannel', 'VariableNames', ...
        {'Channel','Pin','Rest_V','ActiveReference_V','Span_V','Valid'}));
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

function log = emptyLog
    log.time_s=zeros(0,1); log.sequenceNumber=zeros(0,1,'uint16');
    log.deviceStatus=zeros(0,1,'uint8'); log.frameValid=false(0,2);
    log.accelRaw_g=zeros(0,3,2); log.accelCalibrated_g=zeros(0,3,2);
    log.gyroCorrected_rad_s=zeros(0,3,2); log.temperature_C=zeros(0,2);
    log.yawPitchRoll_deg=zeros(0,3,2); log.emgVoltage_V=zeros(0,4);
    log.emgNormalized=zeros(0,4); log.imuCandidateHit=false(0,2);
    log.emgConfirmedHit=false(0,2); log.fusionAcceptedHit=false(0,2);
    log.fusionRejectedHit=false(0,2); log.fusionPendingActive=false(0,2);
    log.emgConfirmationScore=zeros(0,2);
    log.fusionDecisionCode=zeros(0,2,'uint8');
    log.candidateTargetIndex=zeros(0,2,'uint8');
    log.targetIndex=zeros(0,2,'uint8');
end

function log = appendLog(log, frames, row, raw, calibrated, gyro, ypr, ...
        volts, profile, candidate, decision, candidateTarget, target)
    at = numel(log.time_s) + 1;
    log.time_s(at,1)=frames.time_s(row); log.sequenceNumber(at,1)=frames.sequence(row);
    log.deviceStatus(at,1)=frames.status(row); log.frameValid(at,:)=frames.frameValid(row,:);
    log.accelRaw_g(at,:,:)=raw; log.accelCalibrated_g(at,:,:)=calibrated;
    log.gyroCorrected_rad_s(at,:,:)=gyro;
    log.temperature_C(at,:)=frames.temperatureC(row,:); log.yawPitchRoll_deg(at,:,:)=ypr;
    log.emgVoltage_V(at,:)=volts;
    log.emgNormalized(at,:)=max(0,(volts-profile.baseline_V)./profile.activeSpan_V);
    log.imuCandidateHit(at,:)=candidate;
    log.emgConfirmedHit(at,:)=decision.accepted;
    log.fusionAcceptedHit(at,:)=decision.accepted;
    log.fusionRejectedHit(at,:)=decision.rejected;
    log.fusionPendingActive(at,:)=decision.pendingActive;
    log.emgConfirmationScore(at,:)=decision.emgScore;
    log.fusionDecisionCode(at,:)=decisionCodes(decision);
    log.candidateTargetIndex(at,:)=candidateTarget;
    log.targetIndex(at,:)=target;
end

function codes = decisionCodes(decision)
    codes = zeros(1,2,'uint8');
    reasons = ["ACCEPTED", "EMG_TIMEOUT", "UNCERTAIN_ORIENTATION", ...
        "DUPLICATE_GUARD", "PENDING_ALREADY_ACTIVE"];
    for hand = 1:2
        match = find(reasons == decision.reason(hand), 1);
        if ~isempty(match), codes(hand) = uint8(match); end
    end
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
    metadata.fusionDecisionCodes=struct('none',uint8(0), ...
        'accepted',uint8(1),'emgTimeout',uint8(2), ...
        'uncertainOrientation',uint8(3),'duplicateGuard',uint8(4), ...
        'pendingAlreadyActive',uint8(5));
    metadata.timestampSource="Arduino micros() shared by both IMUs and all EMG channels";
    metadata.completedNormally=true;
end
