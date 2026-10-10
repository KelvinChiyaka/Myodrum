function mainDualKickIMU
% MAINDUALKICKIMU Calibrate, orient, log, and play kick from both IMUs.
%
% IMU 1 at 0x68 and IMU 2 at 0x69 each run an independent copy of the
% proven kick detector. Six-pose accelerometer calibration is persistent;
% gyro bias is measured every session. Two independent imufilter instances
% estimate relative yaw, pitch, and roll for logging and live plots.

    clc;
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(projectRoot, 'common'));
    addpath(fullfile(projectRoot, 'IMU_subsystem'));
    addpath(fullfile(projectRoot, 'calibration'));
    addpath(fullfile(projectRoot, 'classifiers', 'ruleBased'));
    addpath(fullfile(projectRoot, 'audio'));
    cfg = myoDrumConfig(projectRoot);

    if exist('imufilter', 'class') ~= 8
        error('MyoDrum:MissingIMUFilter', ...
            ['imufilter is unavailable. Install or license Sensor Fusion ' ...
             'and Tracking Toolbox.']);
    end

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
    metadata.mode = "dual-kick";
    metadata.target = "KICK";
    metadata.config = cfg;
    metadata.rawDataFile = string(rawFile);
    metadata.calibrationFile = string(fullfile( ...
        metadataFolder, 'imuSessionCalibration.mat'));
    metadata.completedNormally = false;
    save(metadataFile, 'metadata');

    fprintf('MyoDrum: calibrated two-IMU kick session\n');
    fprintf('Session: %s\n', sessionId);
    fprintf('Connecting to %s on %s...\n', ...
        cfg.Hardware.Board, cfg.Hardware.COMPort);
    board = arduino(cfg.Hardware.COMPort, cfg.Hardware.Board, ...
        'Libraries', 'I2C');
    sensors = setupMPU6050Pair( ...
        board, cfg.IMU.Addresses, cfg.IMU.SensorOutputRateHz);
    sampleRateHz = measureDualIMUReadRate(sensors, 25);
    metadata.measuredSampleRateHz = sampleRateHz;
    save(metadataFile, 'metadata');

    calibration = calibrateDualIMUSession( ...
        sensors, sampleRateHz, cfg, metadataFolder);

    filters = cell(1, 2);
    for sensorIndex = 1:2
        filters{sensorIndex} = imufilter( ...
            'SampleRate', sampleRateHz, ...
            'ReferenceFrame', char(cfg.Orientation.ReferenceFrame));
    end

    fprintf('\nORIENTATION FILTER WARM-UP\n');
    fprintf(['Hold both sticks still in a comfortable neutral playing pose. ' ...
        'Warming up for %.1f seconds...\n'], cfg.Orientation.WarmupSeconds);
    warmupCount = max(20, round( ...
        sampleRateHz * cfg.Orientation.WarmupSeconds));
    neutralYprDeg = zeros(2, 3);
    for sampleIndex = 1:warmupCount
        for sensorIndex = 1:2
            sample = readMPU6050Sample(sensors(sensorIndex));
            accelG = applyMPU6050AccelCalibration( ...
                sample.accelG, calibration.accel{sensorIndex});
            accelMps2 = accelG * 9.80665;
            gyroRadPerSec = deg2rad(sample.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));
            orientation = filters{sensorIndex}(accelMps2, gyroRadPerSec);
            neutralYprDeg(sensorIndex, :) = ...
                eulerd(orientation, 'ZYX', 'frame');
        end
    end
    fprintf('Orientation filters ready. Angles are relative to this neutral pose.\n');
    fprintf('Yaw is relative and will drift because MPU6050 has no magnetometer.\n');

    players = createKickPlayerBank( ...
        fullfile(cfg.Paths.Audio, 'kick.wav'), cfg.Kick.AudioVoices);
    nextVoice = 1;
    detectors = [initializeKickDetector, initializeKickDetector];
    hitCount = zeros(1, 2);

    [fig, angleLines] = createOrientationFigure(cfg);
    setappdata(fig, 'MyoDrumRunning', true);

    logger = DualIMUSessionLogger(rawFile, metadataFile, metadata, ...
        cfg.Logging.BufferSamples);
    cleanup = onCleanup(@()shutdownDualKick( ...
        logger, filters, players, fig, cfg, metadataFolder));

    fprintf('\nREADY: either stick can play KICK.\n');
    fprintf('Press Q in the plot window or close it to stop and save.\n\n');
    sessionTimer = tic;
    sampleNumber = 0;

    while isgraphics(fig) && getappdata(fig, 'MyoDrumRunning')
        sampleNumber = sampleNumber + 1;
        timeNow = toc(sessionTimer);
        cycleRawAccel = zeros(1, 3, 2);
        cycleCalibratedAccel = zeros(1, 3, 2);
        cycleCorrectedGyro = zeros(1, 3, 2);
        cycleTemperature = zeros(1, 2);
        cycleYpr = zeros(1, 3, 2);
        cycleHit = false(1, 2);

        for sensorIndex = 1:2
            sample = readMPU6050Sample(sensors(sensorIndex));
            calibratedAccelG = applyMPU6050AccelCalibration( ...
                sample.accelG, calibration.accel{sensorIndex});
            accelMps2 = calibratedAccelG * 9.80665;
            correctedGyroRad = deg2rad(sample.gyroDps - ...
                calibration.gyroBiasDps(sensorIndex, :));

            orientation = filters{sensorIndex}( ...
                accelMps2, correctedGyroRad);
            absoluteYprDeg = eulerd(orientation, 'ZYX', 'frame');
            relativeYprDeg = wrapDegrees( ...
                absoluteYprDeg - neutralYprDeg(sensorIndex, :));

            [detectors(sensorIndex), hit, diagnostics] = ...
                updateKickDetector(detectors(sensorIndex), accelMps2, ...
                correctedGyroRad, calibration.directions(sensorIndex), ...
                timeNow, sampleRateHz, cfg.Kick);

            if hit
                play(players{nextVoice});
                nextVoice = mod(nextVoice, numel(players)) + 1;
                hitCount(sensorIndex) = hitCount(sensorIndex) + 1;
                fprintf('%s KICK %d | rate=%+.2f rad/s | jerk=%.2f m/s^2\n', ...
                    cfg.IMU.HandNames(sensorIndex), hitCount(sensorIndex), ...
                    diagnostics.signedRateRadPerSec, ...
                    diagnostics.accelStepMps2);
            end

            cycleRawAccel(1, :, sensorIndex) = sample.accelG;
            cycleCalibratedAccel(1, :, sensorIndex) = ...
                calibratedAccelG;
            cycleCorrectedGyro(1, :, sensorIndex) = ...
                correctedGyroRad;
            cycleTemperature(1, sensorIndex) = sample.temperatureC;
            cycleYpr(1, :, sensorIndex) = relativeYprDeg;
            cycleHit(1, sensorIndex) = hit;

            if mod(sampleNumber, cfg.Orientation.PlotEverySamples) == 0
                for angleIndex = 1:3
                    addpoints(angleLines{sensorIndex}(angleIndex), ...
                        timeNow, relativeYprDeg(angleIndex));
                end
            end
        end

        logger.append(timeNow, cycleRawAccel, cycleCalibratedAccel, ...
            cycleCorrectedGyro, cycleTemperature, cycleYpr, cycleHit);
        if mod(sampleNumber, cfg.Orientation.PlotEverySamples) == 0
            drawnow limitrate;
        end
    end

    logger.CompletedNormally = true;
    clear cleanup; % closes logger and releases hardware-related resources
end

function shutdownDualKick(logger, filters, players, fig, cfg, metadataFolder)
    try
        logger.close();
    catch saveError
        warning('MyoDrum:FinalLogSaveFailed', ...
            'Could not flush the final sensor buffer: %s', saveError.message);
    end
    for filterIndex = 1:numel(filters)
        try
            release(filters{filterIndex});
        catch
        end
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
    fprintf('\nTwo-IMU kick session stopped.\n');
    fprintf('Logged %d paired samples. Hits: %s=%d, %s=%d.\n', ...
        logger.LoggedSamples, cfg.IMU.HandNames(1), logger.KickCount(1), ...
        cfg.IMU.HandNames(2), logger.KickCount(2));
    fprintf('Raw data:\n  %s\n', logger.RawFile);
    fprintf('Metadata and calibration:\n  %s\n', metadataFolder);
end

function [fig, lines] = createOrientationFigure(cfg)
    fig = figure('Name', 'MyoDrum - Two IMU orientation and kick', ...
        'NumberTitle', 'off', 'Color', 'w', ...
        'KeyPressFcn', @keyPressed, 'CloseRequestFcn', @closeRequested);
    colors = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
    names = {'Yaw (relative)', 'Pitch', 'Roll'};
    lines = cell(1, 2);
    for sensorIndex = 1:2
        subplot(2, 1, sensorIndex);
        hold on;
        lines{sensorIndex} = gobjects(1, 3);
        for angleIndex = 1:3
            lines{sensorIndex}(angleIndex) = animatedline( ...
                'Color', colors(angleIndex, :), 'LineWidth', 1.2, ...
                'DisplayName', names{angleIndex}, ...
                'MaximumNumPoints', cfg.Orientation.MaximumPlotPoints);
        end
        grid on;
        xlabel('Time (s)');
        ylabel('Relative angle (degrees)');
        title(sprintf('%s hand - MPU6050 %s', ...
            cfg.IMU.HandNames(sensorIndex), cfg.IMU.Addresses(sensorIndex)));
        legend('show', 'Location', 'best');
        ylim([-180, 180]);
    end
end

function keyPressed(source, event)
    if strcmpi(event.Key, 'q')
        setappdata(source, 'MyoDrumRunning', false);
    end
end

function closeRequested(source, ~)
    setappdata(source, 'MyoDrumRunning', false);
end

function wrapped = wrapDegrees(angleDeg)
    wrapped = mod(angleDeg + 180, 360) - 180;
end
