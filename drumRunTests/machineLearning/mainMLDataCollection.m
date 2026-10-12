function mainMLDataCollection
% MAINMLDATACOLLECTION Record labelled dual-IMU and four-channel EMG blocks.

    clc;
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'classifiers', 'machineLearning'));
    cfg = myoDrumConfig(root);
    [accelCalibration, accelCalibrationFiles] = ...
        loadAccelerometerCalibrations(cfg);
    schedule = buildMLCollectionSchedule(cfg.IMU.HandNames, ...
        cfg.Targets.Names, cfg.MachineLearning.CollectionRounds, ...
        cfg.MachineLearning.HitsPerClassPerHandPerRound, ...
        cfg.MachineLearning.NoHitMinimumSeconds);

    sessionId = "session_" + string(datetime('now', ...
        'Format', 'yyyyMMdd_HHmmss'));
    startedAt = string(datetime('now', 'TimeZone', 'local'));
    rawFolder = fullfile(cfg.Paths.RawData, sessionId);
    metadataFolder = fullfile(cfg.Paths.Metadata, 'sessions', sessionId);
    mkdir(rawFolder);
    mkdir(metadataFolder);
    rawFile = fullfile(rawFolder, 'mlTrainingStream.mat');
    calibrationFile = fullfile(rawFolder, 'emgConfirmationCalibration.mat');
    metadataFile = fullfile(metadataFolder, 'sessionMetadata.mat');
    manifestFile = fullfile(metadataFolder, 'mlCollectionManifest.csv');

    metadata = initialMetadata(sessionId, startedAt, cfg, rawFile, ...
        calibrationFile, manifestFile, schedule, accelCalibrationFiles);
    save(metadataFile, 'metadata');
    log = emptyCollectionLog;
    manifest = emptyManifest(schedule);
    profile = struct;
    gyroBiasDps = nan(2, 3);
    fprintf('MyoDrum: labelled machine-learning data collection\n');
    fprintf('Session: %s\n', sessionId);
    fprintf('Firmware target: %.0f synchronized frames/s.\n', ...
        cfg.EMG.CombinedStreamRateHz);
    fprintf('This records data only; it does not classify or play sounds.\n');
    fprintf('Planned blocks: %d (%d labelled hits plus NO_HIT movement).\n', ...
        height(schedule), sum(double(schedule.ExpectedHitCount)));

    try
        reader = IMUEMGStreamReader(cfg.Hardware.COMPort, ...
            cfg.EMG.CombinedStreamBaud);
        cleanupReader = onCleanup(@()delete(reader));

        fprintf('\nREST/BIAS CALIBRATION\n');
        input(['Keep both sticks and all muscles relaxed, then press ' ...
            'Enter...'], 's');
        reader.start();
        restFrames = collectForSeconds(reader, ...
            max(3, cfg.EMG.RestSeconds), cfg);
        ensureHealthyFrames(restFrames, cfg);
        for hand = 1:2
            gyroBiasDps(hand, :) = median( ...
                restFrames.gyroDps(:, :, hand), 1);
        end
        restVoltage = adcToVolts(restFrames.emgAdc, cfg);

        fig = createCollectionControl;
        cleanupFigure = onCleanup(@()closeFigure(fig));
        activeFrames = emptyFrames;
        activeSampleHands = strings(0, 1);
        for hand = 1:2
            fprintf('\n%s EMG REFERENCE\n', cfg.IMU.HandNames(hand));
            fprintf(['Keep the other arm relaxed. Click START, then make ' ...
                'comfortable KICK, SNARE and CRASH gestures with the %s ' ...
                'hand for %.0f seconds.\n'], cfg.IMU.HandNames(hand), ...
                cfg.EMG.ActiveCalibrationSeconds);
            setReferencePrompt(fig, cfg.IMU.HandNames(hand), ...
                cfg.EMG.ActiveCalibrationSeconds);
            action = waitForStart(reader, fig, cfg);
            if action == "QUIT"
                error('MyoDrum:MLCollectionCancelled', ...
                    'Machine-learning collection cancelled by the operator.');
            end
            discardPendingFrames(reader, cfg);
            setReferenceRecordingPrompt(fig, cfg.IMU.HandNames(hand));
            fprintf('RECORDING %s EMG NOW...\n', cfg.IMU.HandNames(hand));
            trial = collectForSeconds(reader, ...
                cfg.EMG.ActiveCalibrationSeconds, cfg);
            activeFrames = appendFrames(activeFrames, trial);
            activeSampleHands = [activeSampleHands; repmat( ...
                cfg.IMU.HandNames(hand), numel(trial.time_s), 1)]; %#ok<AGROW>
        end
        reader.stop();

        activeVoltage = adcToVolts(activeFrames.emgAdc, cfg);
        profile = buildEMGConfirmationProfile(restVoltage, activeVoltage, ...
            cfg.EMG.Hands, MinimumActiveSpanVolts= ...
            cfg.EMG.MinimumActiveSpanVolts, ...
            ActiveSampleHands=activeSampleHands, ...
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
        save(calibrationFile, 'emgCalibration', '-v7.3');
        fprintf('EMG calibration capture saved:\n  %s\n', calibrationFile);
        printEMGProfile(profile, cfg);
        requireSafeEMGProfile(profile, cfg);

        fprintf(['\nCOLLECTION CONTROLS\nUse the control window: START begins ' ...
            'a block, FINISH ends it, and Q stops safely.\n']);
        fprintf(['Hit blocks request exactly %d gestures. NO_HIT blocks ' ...
            'request normal non-striking movement.\n\n'], ...
            cfg.MachineLearning.HitsPerClassPerHandPerRound);

        for blockRow = 1:height(schedule)
            plan = schedule(blockRow, :);
            printBlockPrompt(plan, height(schedule));
            setControlPrompt(fig, plan, blockRow, height(schedule), "WAITING");
            reader.start();
            action = waitForStart(reader, fig, cfg);
            if action == "QUIT"
                reader.stop();
                break;
            end
            discardPendingFrames(reader, cfg);
            setControlPrompt(fig, plan, blockRow, height(schedule), "RECORDING");
            [frames, completed] = collectBlock(reader, fig, plan, cfg);
            reader.stop();
            if ~isempty(frames.sequence)
                log = appendCollectionBlock(log, frames, plan, ...
                    accelCalibration, gyroBiasDps, profile, cfg);
                manifest = appendManifestRow(manifest, plan, frames, completed);
            end
            metadata.completedBlocks = height(manifest);
            metadata.loggedSamples = numel(log.time_s);
            metadata.gyroBiasDps = gyroBiasDps;
            metadata.emgProfile = profile;
            saveCheckpoint(rawFile, manifestFile, metadataFile, log, ...
                manifest, cfg.MachineLearning.ClassNames, metadata);
            if ~completed
                break;
            end
            fprintf('Saved block %d/%d: %s %s (%.1f s).\n', ...
                blockRow, height(schedule), plan.Hand, plan.Label, ...
                frames.time_s(end) - frames.time_s(1));
        end

        statistics = reader.getStatistics();
        metadata.endedAt = string(datetime('now', 'TimeZone', 'local'));
        metadata.streamStatistics = statistics;
        metadata.completedBlocks = height(manifest);
        metadata.loggedSamples = numel(log.time_s);
        metadata.gyroBiasDps = gyroBiasDps;
        metadata.emgProfile = profile;
        metadata.completedNormally = height(manifest) == height(schedule) && ...
            all(manifest.Completed);
        saveCheckpoint(rawFile, manifestFile, metadataFile, log, manifest, ...
            cfg.MachineLearning.ClassNames, metadata);
        fprintf('\nML collection saved: %d/%d blocks, %d frames.\n', ...
            height(manifest), height(schedule), numel(log.time_s));
        fprintf('Raw data:\n  %s\nManifest:\n  %s\n', ...
            rawFile, manifestFile);
    catch exception
        if exist('reader', 'var') && ~isempty(reader)
            try
                reader.stop();
                metadata.streamStatistics = reader.getStatistics();
            catch
            end
        end
        metadata.endedAt = string(datetime('now', 'TimeZone', 'local'));
        metadata.completedNormally = false;
        metadata.completedBlocks = height(manifest);
        metadata.loggedSamples = numel(log.time_s);
        metadata.failureIdentifier = string(exception.identifier);
        metadata.failureMessage = string(exception.message);
        if ~isempty(fieldnames(profile))
            metadata.emgProfile = profile;
        end
        metadata.gyroBiasDps = gyroBiasDps;
        saveCheckpoint(rawFile, manifestFile, metadataFile, log, manifest, ...
            cfg.MachineLearning.ClassNames, metadata);
        rethrow(exception);
    end
end

function metadata = initialMetadata(id, startedAt, cfg, rawFile, ...
        calibrationFile, manifestFile, schedule, accelFiles)
    metadata.sessionId = id;
    metadata.startedAt = startedAt;
    metadata.mode = "ml-data-collection";
    metadata.config = cfg;
    metadata.rawDataFile = string(rawFile);
    metadata.emgCalibrationFile = string(calibrationFile);
    metadata.manifestFile = string(manifestFile);
    metadata.plannedSchedule = schedule;
    metadata.accelerometerCalibrationFiles = string(accelFiles);
    metadata.timestampSource = ...
        "Arduino micros() shared by both IMUs and all EMG channels";
    metadata.labelMeaning = [ ...
        "Labels describe the prompted block, not every individual frame. ", ...
        "Feature extraction must locate movement windows inside each block."];
    metadata.completedNormally = false;
    metadata.completedBlocks = 0;
    metadata.loggedSamples = 0;
end

function [calibrations, files] = loadAccelerometerCalibrations(cfg)
    folder = fullfile(cfg.Paths.Metadata, 'calibration');
    calibrations = cell(1, 2);
    files = strings(2, 1);
    for hand = 1:2
        token = erase(cfg.IMU.Addresses(hand), '0x');
        files(hand) = fullfile(folder, "mpu6050_0x" + token + ...
            "_accelCalibration.mat");
        if ~isfile(files(hand))
            error('MyoDrum:MissingMLCollectionCalibration', ...
                ['Required accelerometer calibration is missing: %s. ' ...
                'Run the IMU-only workflow first.'], files(hand));
        end
        saved = load(files(hand), 'calibration');
        calibrations{hand} = saved.calibration;
    end
end

function action = waitForStart(reader, fig, cfg)
    setappdata(fig, 'MyoDrumAction', "");
    lastFrameTimer = tic;
    restartAttempted = false;
    while isgraphics(fig)
        drawnow limitrate;
        action = string(getappdata(fig, 'MyoDrumAction'));
        if action == "START" || action == "QUIT"
            return;
        end
        frames = reader.readAvailableFrames();
        if ~isempty(frames.sequence)
            lastFrameTimer = tic;
        elseif toc(lastFrameTimer) > cfg.EMG.StreamTimeoutSeconds
            if ~restartAttempted
                warning('MyoDrum:MLCollectionStreamRestart', [ ...
                    'The synchronized stream paused while waiting. ' ...
                    'Restarting it once automatically.']);
                reader.start();
                restartAttempted = true;
                lastFrameTimer = tic;
                continue;
            end
            error('MyoDrum:MLCollectionStreamTimeout', [ ...
                'No synchronized frames arrived after an automatic ' ...
                'restart. Reconnect the Mega and retry.']);
        end
        pause(0.001);
    end
    action = "QUIT";
end

function [frames, completed] = collectBlock(reader, fig, plan, cfg)
    frames = emptyFrames;
    completed = false;
    setappdata(fig, 'MyoDrumAction', "");
    startTime = [];
    lastFrameTimer = tic;
    earlyFinishReported = false;
    while isgraphics(fig)
        drawnow limitrate;
        available = reader.readAvailableFrames();
        if ~isempty(available.sequence)
            if isempty(startTime), startTime = available.time_s(1); end
            frames = appendFrames(frames, available);
            lastFrameTimer = tic;
        elseif toc(lastFrameTimer) > cfg.EMG.StreamTimeoutSeconds
            error('MyoDrum:MLCollectionStreamTimeout', ...
                'No synchronized frames arrived for %.1f seconds.', ...
                cfg.EMG.StreamTimeoutSeconds);
        end
        if isempty(frames.time_s)
            elapsed = 0;
        else
            elapsed = frames.time_s(end) - startTime;
        end
        updateElapsedTime(fig, elapsed);
        action = string(getappdata(fig, 'MyoDrumAction'));
        if action == "QUIT"
            return;
        elseif action == "FINISH"
            if elapsed >= plan.MinimumDuration_s
                completed = true;
                return;
            end
            if ~earlyFinishReported
                fprintf(['Block needs at least %.1f seconds; continue for ' ...
                    'another %.1f seconds.\n'], plan.MinimumDuration_s, ...
                    plan.MinimumDuration_s - elapsed);
                earlyFinishReported = true;
            end
            setappdata(fig, 'MyoDrumAction', "");
        end
        pause(0.001);
    end
end

function printBlockPrompt(plan, blockCount)
    fprintf('\nBLOCK %d/%d | ROUND %d | %s | %s\n', ...
        plan.BlockId, blockCount, plan.Round, plan.Hand, plan.Label);
    if plan.Label == "NO_HIT"
        fprintf(['Move the %s stick naturally without intending a drum hit. ' ...
            'Include repositioning, upward-only motion and a muscle-only ' ...
            'squeeze. Record at least %.0f seconds.\n'], ...
            plan.Hand, plan.MinimumDuration_s);
    else
        fprintf(['Use only the %s hand and perform exactly %d comfortable ' ...
            '%s gestures. Keep the other hand relaxed.\n'], ...
            plan.Hand, plan.ExpectedHitCount, plan.Label);
    end
    fprintf('Click START. Click FINISH (or press Enter/N) when done.\n');
end

function fig = createCollectionControl
    fig = figure('Name', 'MyoDrum ML data collection', ...
        'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
        'Position', [400 300 620 230], 'KeyPressFcn', @keyPressed, ...
        'CloseRequestFcn', @closeRequested);
    instruction = uicontrol(fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.05 0.55 0.90 0.36], ...
        'FontSize', 13, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'String', 'Preparing...');
    elapsed = uicontrol(fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.35 0.42 0.30 0.10], ...
        'FontSize', 11, 'String', 'Elapsed: 0.0 s');
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'START', ...
        'Units', 'normalized', 'Position', [0.08 0.12 0.24 0.20], ...
        'FontSize', 12, 'Callback', ...
        @(~,~)setCollectionAction(fig, "START"));
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'FINISH', ...
        'Units', 'normalized', 'Position', [0.38 0.12 0.24 0.20], ...
        'FontSize', 12, 'Callback', ...
        @(~,~)setCollectionAction(fig, "FINISH"));
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'STOP AND SAVE', ...
        'Units', 'normalized', 'Position', [0.68 0.12 0.24 0.20], ...
        'FontSize', 12, 'Callback', ...
        @(~,~)setCollectionAction(fig, "QUIT"));
    setappdata(fig, 'MyoDrumAction', "");
    setappdata(fig, 'MyoDrumStage', "WAITING");
    setappdata(fig, 'InstructionHandle', instruction);
    setappdata(fig, 'ElapsedHandle', elapsed);
end

function setControlPrompt(fig, plan, row, total, stage)
    setappdata(fig, 'MyoDrumStage', stage);
    setappdata(fig, 'MyoDrumAction', "");
    instruction = getappdata(fig, 'InstructionHandle');
    if stage == "WAITING"
        prefix = "READY";
    else
        prefix = "RECORDING";
    end
    if isgraphics(instruction)
        instruction.String = sprintf('%s | Block %d/%d | Round %d | %s %s', ...
            prefix, row, total, plan.Round, plan.Hand, plan.Label);
    end
    updateElapsedTime(fig, 0);
end

function setReferencePrompt(fig, handName, durationSeconds)
    setappdata(fig, 'MyoDrumStage', "WAITING");
    setappdata(fig, 'MyoDrumAction', "");
    instruction = getappdata(fig, 'InstructionHandle');
    if isgraphics(instruction)
        instruction.String = sprintf( ...
            'READY | %s EMG reference | %.0f seconds', ...
            handName, durationSeconds);
    end
    updateElapsedTime(fig, 0);
end

function setReferenceRecordingPrompt(fig, handName)
    setappdata(fig, 'MyoDrumStage', "REFERENCE");
    setappdata(fig, 'MyoDrumAction', "");
    instruction = getappdata(fig, 'InstructionHandle');
    if isgraphics(instruction)
        instruction.String = sprintf('RECORDING | %s EMG reference', handName);
    end
end

function updateElapsedTime(fig, elapsed)
    handle = getappdata(fig, 'ElapsedHandle');
    if isgraphics(handle)
        handle.String = sprintf('Elapsed: %.1f s', elapsed);
    end
end

function keyPressed(fig, event)
    switch lower(event.Key)
        case {'q', 'escape'}
            setCollectionAction(fig, "QUIT");
        case {'n', 'return'}
            if string(getappdata(fig, 'MyoDrumStage')) == "RECORDING"
                setCollectionAction(fig, "FINISH");
            else
                setCollectionAction(fig, "START");
            end
        case {'s', 'space'}
            setCollectionAction(fig, "START");
    end
end

function closeRequested(fig, ~)
    setCollectionAction(fig, "QUIT");
    set(fig, 'Visible', 'off');
end

function setCollectionAction(fig, action)
    if isgraphics(fig)
        setappdata(fig, 'MyoDrumAction', action);
    end
end

function closeFigure(fig)
    if isgraphics(fig), delete(fig); end
end

function log = appendCollectionBlock(log, frames, plan, ...
        accelCalibration, gyroBiasDps, profile, cfg)
    block = prepareMLCollectionBlock(frames, plan, accelCalibration, ...
        gyroBiasDps, profile, cfg);
    for name = string(fieldnames(log)).'
        log.(name) = cat(1, log.(name), block.(name));
    end
end

function manifest = appendManifestRow(manifest, plan, frames, completed)
    row = plan;
    row.StartTime_s = frames.time_s(1);
    row.EndTime_s = frames.time_s(end);
    row.Duration_s = frames.time_s(end) - frames.time_s(1);
    row.RecordedFrames = uint32(numel(frames.time_s));
    row.Completed = logical(completed);
    manifest = [manifest; row];
end

function manifest = emptyManifest(schedule)
    manifest = schedule([],:);
    manifest.StartTime_s = zeros(0,1);
    manifest.EndTime_s = zeros(0,1);
    manifest.Duration_s = zeros(0,1);
    manifest.RecordedFrames = zeros(0,1,'uint32');
    manifest.Completed = false(0,1);
end

function saveCheckpoint(rawFile, manifestFile, metadataFile, log, ...
        manifest, classNames, metadata)
    payload = log;
    payload.blockManifest = manifest;
    payload.classNames = classNames;
    save(rawFile, '-struct', 'payload', '-v7.3');
    writetable(manifest, manifestFile);
    save(metadataFile, 'metadata');
end

function log = emptyCollectionLog
    log.time_s = zeros(0,1);
    log.timestamp_us = zeros(0,1,'uint32');
    log.sequenceNumber = zeros(0,1,'uint16');
    log.deviceStatus = zeros(0,1,'uint8');
    log.frameValid = false(0,2);
    log.accelRaw_g = zeros(0,3,2);
    log.accelCalibrated_g = zeros(0,3,2);
    log.gyroRaw_dps = zeros(0,3,2);
    log.gyroCorrected_rad_s = zeros(0,3,2);
    log.temperature_C = zeros(0,2);
    log.emgAdc = zeros(0,4,'uint16');
    log.emgVoltage_V = zeros(0,4);
    log.emgNormalized = zeros(0,4);
    log.blockId = zeros(0,1,'uint16');
    log.round = zeros(0,1,'uint8');
    log.handIndex = zeros(0,1,'uint8');
    log.blockClassIndex = zeros(0,1,'uint8');
end

function frames = collectForSeconds(reader, seconds, cfg)
    frames = emptyFrames;
    startTime = [];
    phaseTimer = tic;
    lastFrameTimer = tic;
    restartAttempted = false;
    while isempty(startTime) || isempty(frames.time_s) || ...
            frames.time_s(end) - startTime < seconds
        available = reader.readAvailableFrames();
        if isempty(available.sequence)
            if toc(lastFrameTimer) > cfg.EMG.StreamTimeoutSeconds
                if ~restartAttempted
                    warning('MyoDrum:MLCollectionStreamRestart', [ ...
                        'The synchronized stream paused during calibration. ' ...
                        'Restarting this capture once automatically.']);
                    reader.start();
                    frames = emptyFrames;
                    startTime = [];
                    phaseTimer = tic;
                    lastFrameTimer = tic;
                    restartAttempted = true;
                    continue;
                end
                error('MyoDrum:MLCollectionStreamTimeout', [ ...
                    'No synchronized frames arrived after an automatic ' ...
                    'restart. Reconnect the Mega and retry.']);
            end
            pause(0.001);
            continue;
        end
        lastFrameTimer = tic;
        if isempty(startTime), startTime = available.time_s(1); end
        frames = appendFrames(frames, available);
        if toc(phaseTimer) > seconds + cfg.EMG.StreamTimeoutSeconds
            error('MyoDrum:MLCollectionStreamTooSlow', ...
                'The stream did not provide %.1f seconds of data in time.', ...
                seconds);
        end
    end
end

function discardPendingFrames(reader, cfg)
    timer = tic;
    while true
        pending = reader.readAvailableFrames();
        if isempty(pending.sequence), return; end
        if toc(timer) > cfg.EMG.StreamTimeoutSeconds
            error('MyoDrum:MLCollectionStreamBacklog', ...
                'MATLAB could not catch up with the synchronized stream.');
        end
    end
end

function frames = emptyFrames
    frames.sequence = zeros(0,1,'uint16');
    frames.timestamp_us = zeros(0,1,'uint32');
    frames.time_s = zeros(0,1);
    frames.accelG = zeros(0,3,2);
    frames.gyroDps = zeros(0,3,2);
    frames.temperatureC = zeros(0,2);
    frames.emgAdc = zeros(0,4,'uint16');
    frames.status = zeros(0,1,'uint8');
    frames.frameValid = false(0,2);
end

function combined = appendFrames(combined, addition)
    for name = string(fieldnames(combined)).'
        combined.(name) = cat(1, combined.(name), addition.(name));
    end
end

function ensureHealthyFrames(frames, cfg)
    if numel(frames.sequence) < cfg.EMG.CombinedStreamRateHz
        error('MyoDrum:InsufficientMLCollectionFrames', ...
            'Too few valid synchronized frames arrived.');
    end
    if any(mean(frames.frameValid, 1) < 0.99)
        error('MyoDrum:MLCollectionIMUReadFailures', ...
            'At least one MPU6050 failed more than 1%% of stream reads.');
    end
end

function volts = adcToVolts(adc, cfg)
    volts = double(adc) * cfg.EMG.ADCReferenceVolts / cfg.EMG.ADCMaxCount;
end

function printEMGProfile(profile, cfg)
    disp(table(cfg.EMG.ChannelNames, cfg.EMG.AnalogPins, ...
        profile.baseline_V', profile.noiseSigma_V', ...
        profile.activeReference_V', profile.activeSpan_V', ...
        profile.signalToNoiseRatio', profile.validChannel', ...
        profile.qualityReason', 'VariableNames', ...
        {'Channel','Pin','Rest_V','RestNoise_V','ActiveReference_V', ...
        'Span_V','SignalToNoise','Valid','Reason'}));
end

function requireSafeEMGProfile(profile, cfg)
    if all(profile.validChannel), return; end
    invalid = ~profile.validChannel(:);
    channelNames = cfg.EMG.ChannelNames(:);
    reasons = profile.qualityReason(:);
    details = channelNames(invalid) + " (" + reasons(invalid) + ")";
    error('MyoDrum:UnsafeMLCollectionEMG', ...
        ['EMG hardware check failed: %s. The labelled collection was not ' ...
        'started. Check power, common ground, electrodes and amplifier gain.'], ...
        strjoin(details, ', '));
end
