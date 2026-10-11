function calibration = calibrateDualIMUSession( ...
        sensors, sampleRateHz, cfg, sessionMetadataFolder)
% CALIBRATEDUALIMUSESSION Calibrate both sensors for one playing session.
%
% Persistent six-pose and direction calibrations are stored in
% data/metadata/calibration. A fresh gyro bias is measured and saved in the
% session metadata folder every time this function runs.

    arguments
        sensors (1,2) struct
        sampleRateHz (1,1) double {mustBeFinite, mustBePositive}
        cfg (1,1) struct
        sessionMetadataFolder (1,1) string
    end

    if ~isfolder(sessionMetadataFolder)
        mkdir(sessionMetadataFolder);
    end
    persistentFolder = fullfile(cfg.Paths.Metadata, 'calibration');
    if ~isfolder(persistentFolder)
        mkdir(persistentFolder);
    end

    calibration.accel = cell(1, 2);
    for sensorIndex = 1:2
        addressToken = erase(cfg.IMU.Addresses(sensorIndex), '0x');
        calibrationFile = fullfile(persistentFolder, ...
            "mpu6050_0x" + addressToken + "_accelCalibration.mat");
        fprintf('\nAccelerometer calibration for %s hand (%s)\n', ...
            cfg.IMU.HandNames(sensorIndex), cfg.IMU.Addresses(sensorIndex));
        calibration.accel{sensorIndex} = ...
            loadOrRunMPU6050AccelCalibration( ...
            @()readMPU6050Sample(sensors(sensorIndex)), sampleRateHz, ...
            calibrationFile, cfg.Calibration.ForceSixPose, ...
            cfg.Calibration.SixPoseSecondsPerPose);
    end

    staticFile = fullfile(persistentFolder, ...
        'twoIMUStaticGyroCalibration.mat');
    [calibration.gyroBiasDps, calibration.gyroNoiseDps, ...
        calibration.gyroBiasSource] = resolveGyroBias( ...
        @()measureSessionGyroBias(sensors, sampleRateHz, ...
            cfg.Calibration.GyroBiasSeconds), staticFile, ...
        cfg.Calibration.GyroBiasPolicy, ...
        cfg.Calibration.GyroBiasMaxAgeHours, ...
        cfg.Calibration.ForceStatic);
    calibration.gyroBiasPolicy = cfg.Calibration.GyroBiasPolicy;

    directionFile = fullfile(persistentFolder, ...
        'twoIMUKickDirectionCalibration.mat');
    calibration.directions = loadOrLearnDirections( ...
        sensors, calibration.gyroBiasDps, sampleRateHz, cfg, directionFile);
    calibration.sampleRateHz = sampleRateHz;
    calibration.handNames = cfg.IMU.HandNames;
    calibration.addresses = cfg.IMU.Addresses;
    calibration.createdAt = string(datetime('now', 'TimeZone', 'local'));
    calibration.schemaVersion = "1.0";

    sessionFile = fullfile(sessionMetadataFolder, 'imuSessionCalibration.mat');
    save(sessionFile, 'calibration');
    fprintf('\nSaved this session''s complete IMU calibration:\n  %s\n', ...
        sessionFile);
end

function [biases, noises] = measureSessionGyroBias(sensors, sampleRateHz, seconds)
    fprintf('\nSTATIC GYROSCOPE CALIBRATION\n');
    fprintf('Keep BOTH sticks completely still. Calibration starts in 2 seconds.\n');
    countdownCycles = max(1, round(2 * sampleRateHz));
    for cycleIndex = 1:countdownCycles
        readMPU6050Sample(sensors(1));
        readMPU6050Sample(sensors(2));
    end
    fprintf('Calibrating now; keep both sticks still...\n');

    sampleCount = max(20, round(seconds * sampleRateHz));
    gyro = zeros(sampleCount, 3, 2);
    for sampleIndex = 1:sampleCount
        for sensorIndex = 1:2
            sample = readMPU6050Sample(sensors(sensorIndex));
            gyro(sampleIndex, :, sensorIndex) = sample.gyroDps;
        end
    end

    biases = zeros(2, 3);
    noises = zeros(2, 3);
    for sensorIndex = 1:2
        values = gyro(:, :, sensorIndex);
        biases(sensorIndex, :) = median(values, 1);
        noises(sensorIndex, :) = 1.4826 * ...
            median(abs(values - biases(sensorIndex, :)), 1);
        fprintf('IMU %d bias: [%+.3f %+.3f %+.3f] deg/s\n', ...
            sensorIndex, biases(sensorIndex, :));
        fprintf('IMU %d noise: [ %.3f  %.3f  %.3f] deg/s\n', ...
            sensorIndex, noises(sensorIndex, :));
        if any(noises(sensorIndex, :) > 5)
            error('MyoDrum:GyroMovedDuringCalibration', ...
                'IMU %d moved substantially during static calibration.', ...
                sensorIndex);
        end
    end
end

function directions = loadOrLearnDirections( ...
        sensors, gyroBiasDps, sampleRateHz, cfg, directionFile)
    if ~cfg.Calibration.ForceDirection && isfile(directionFile)
        loaded = load(directionFile, 'directions');
        if isfield(loaded, 'directions') && directionsAreValid(loaded.directions)
            directions = loaded.directions;
            fprintf('Loaded saved downstroke directions for both IMUs.\n');
            return;
        end
    end

    template = struct('axis', 0, 'axisName', "", 'sign', 0);
    directions = repmat(template, 1, 2);
    for sensorIndex = 1:2
        fprintf('\nDIRECTION SETUP: %s hand (%s)\n', ...
            cfg.IMU.HandNames(sensorIndex), cfg.IMU.Addresses(sensorIndex));
        fprintf('Hold this stick at the top and keep the other stick still.\n');
        for remaining = 3:-1:1
            fprintf('%d...\n', remaining);
            for cycleIndex = 1:max(1, round(sampleRateHz))
                readMPU6050Sample(sensors(1));
                readMPU6050Sample(sensors(2));
            end
        end
        fprintf('GO: make one normal downward wrist stroke now.\n');

        sampleCount = max(20, round(2.5 * sampleRateHz));
        gyroRad = zeros(sampleCount, 3);
        for sampleIndex = 1:sampleCount
            sample = readMPU6050Sample(sensors(sensorIndex));
            gyroRad(sampleIndex, :) = deg2rad( ...
                sample.gyroDps - gyroBiasDps(sensorIndex, :));
        end

        [~, axisIndex] = max(sum(gyroRad.^2, 1));
        axisSignal = gyroRad(:, axisIndex);
        peakRate = max(abs(axisSignal));
        if peakRate < cfg.Calibration.DirectionMinimumPeakRadPerSec
            error('MyoDrum:DirectionStrokeTooSmall', ...
                ['No clear stroke was measured for IMU %d (peak %.3f rad/s). ' ...
                 'Delete/repeat direction calibration and use one normal stroke.'], ...
                sensorIndex, peakRate);
        end

        strong = abs(axisSignal) >= 0.65 * peakRate;
        signedMotion = median(axisSignal(strong));
        if abs(signedMotion) < 0.05
            [~, peakIndex] = max(abs(axisSignal));
            signedMotion = axisSignal(peakIndex);
        end
        axisNames = ["X", "Y", "Z"];
        directions(sensorIndex).axis = axisIndex;
        directions(sensorIndex).axisName = axisNames(axisIndex);
        directions(sensorIndex).sign = sign(signedMotion);
        fprintf('Learned %s gyro, sign %+d for %s hand.\n', ...
            directions(sensorIndex).axisName, ...
            directions(sensorIndex).sign, cfg.IMU.HandNames(sensorIndex));
    end
    save(directionFile, 'directions');
end

function valid = directionsAreValid(directions)
    valid = numel(directions) == 2;
    if ~valid
        return;
    end
    for sensorIndex = 1:2
        valid = valid && isfield(directions(sensorIndex), 'axis') && ...
            isfield(directions(sensorIndex), 'axisName') && ...
            isfield(directions(sensorIndex), 'sign') && ...
            ismember(directions(sensorIndex).axis, 1:3) && ...
            ismember(directions(sensorIndex).sign, [-1, 1]);
    end
end
