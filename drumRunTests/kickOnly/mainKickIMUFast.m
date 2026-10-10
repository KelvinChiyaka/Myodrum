function mainKickIMUFast
% MAINKICKIMUFAST Restored fast one-IMU kick detector.
% Uses the original direct MPU6050 decoder and signed gyro reversal method.

    clc;

    %% SETTINGS USED BY THE WORKING VERSION
    cfg.COMPort = "COM7";
    cfg.Board = "Mega2560";
    cfg.SensorOutputRate = 50;
    cfg.Gravity = 9.80665;

    cfg.DownstrokeArmRadPerSec = 0.50;
    cfg.BottomReversalRadPerSec = -0.10;
    cfg.AccelStepThresholdMps2 = 2.5;
    cfg.MissedHitResetRadPerSec = -0.60;

    % Safeguard added after the working version produced duplicate sounds.
    cfg.MinReturnTravelRad = deg2rad(8);
    cfg.MinInterHitSeconds = 0.08;

    cfg.DirectionCalibrationSeconds = 2.5;
    cfg.DirectionMinimumPeakRadPerSec = 0.30;
    cfg.GyroBiasSeconds = 1.5;
    cfg.AudioVoices = 4;

    % Six-pose accelerometer calibration is saved and reused. Set this true
    % only when the IMU mounting changes or the saved calibration is suspect.
    cfg.ForceSixPoseCalibration = false;
    cfg.SixPoseSecondsPerPose = 2.0;

    thisFolder = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(fileparts(thisFolder));
    imuFolder = fullfile(projectRoot, 'IMU_subsystem');
    addpath(imuFolder);
    directionFile = fullfile(thisFolder, 'kickDirectionCalibration.mat');
    accelCalibrationFile = fullfile(projectRoot, 'data', 'metadata', ...
        'mpu6050_0x68_accelCalibration.mat');

    %% CONNECT USING THE ORIGINAL MPU6050 CONFIGURATION
    fprintf('MyoDrum: restored high-speed kick detector\n');
    fprintf('Connecting to %s on %s...\n', cfg.Board, cfg.COMPort);
    board = arduino(cfg.COMPort, cfg.Board, 'Libraries', 'I2C');
    imu = setupMPU6050(board, cfg.SensorOutputRate);
    sampleRate = measureReadRate(imu, 25);

    %% SENSOR CALIBRATION
    % Six-pose calibration estimates accelerometer offset/scale. It is loaded
    % on normal runs, so the six poses do not delay every playing session.
    accelCalibration = loadOrRunMPU6050AccelCalibration( ...
        @()readMPU6050(imu), sampleRate, accelCalibrationFile, ...
        cfg.ForceSixPoseCalibration, cfg.SixPoseSecondsPerPose);

    % Gyro bias is measured at every startup because it changes with sensor
    % temperature. This is the session's static gyroscope calibration.
    gyroBiasDps = measureGyroBias( ...
        imu, sampleRate, cfg.GyroBiasSeconds);

    if isfile(directionFile)
        loaded = load(directionFile, 'direction');
        direction = loaded.direction;
        validateDirection(direction);
        fprintf('Loaded downward direction: %s gyro, sign %+d.\n', ...
            direction.axisName, direction.sign);
    else
        direction = learnDownstrokeDirection( ...
            imu, gyroBiasDps, sampleRate, ...
            cfg.DirectionCalibrationSeconds, ...
            cfg.DirectionMinimumPeakRadPerSec, directionFile);
    end

    %% KICK SOUND
    players = createKickPlayers(cfg.AudioVoices);
    nextVoice = 1;

    %% FAST HIT DETECTOR
    WAIT_DOWNSTROKE = 1;
    WAIT_BOTTOM_REVERSAL = 2;
    WAIT_RETURN_TRAVEL = 3;
    state = WAIT_DOWNSTROKE;

    previousAccel = [];
    recentAccelSteps = zeros(1, 3);
    returnTravelRad = 0;
    lastHitTime = -inf;
    hitCount = 0;

    cleanup = onCleanup(@()shutdownAll(players));
    fprintf('\nREADY. Play repeated down/up strokes. Press Ctrl+C to stop.\n\n');

    timer = tic;
    previousSampleTime = 0;

    while true
        sample = readMPU6050(imu);
        sampleTime = toc(timer);
        samplePeriod = sampleTime - previousSampleTime;
        previousSampleTime = sampleTime;

        % Prevent an I2C recovery pause from counting as real return travel.
        samplePeriod = min(max(samplePeriod, 0), 2 / sampleRate);

        calibratedAccelG = applyMPU6050AccelCalibration( ...
            sample.accelG, accelCalibration);
        accelMps2 = calibratedAccelG * cfg.Gravity;
        gyroRad = deg2rad(sample.gyroDps - gyroBiasDps);
        signedRate = direction.sign * gyroRad(direction.axis);

        if isempty(previousAccel)
            accelStep = 0;
        else
            accelStep = norm(accelMps2 - previousAccel);
        end
        previousAccel = accelMps2;
        recentAccelSteps = [recentAccelSteps(2:3), accelStep];

        if state == WAIT_DOWNSTROKE
            if signedRate >= cfg.DownstrokeArmRadPerSec
                state = WAIT_BOTTOM_REVERSAL;
                recentAccelSteps(:) = 0;
            end

        elseif state == WAIT_BOTTOM_REVERSAL
            changedToUpward = ...
                signedRate <= cfg.BottomReversalRadPerSec;
            bottomJerk = ...
                max(recentAccelSteps) >= cfg.AccelStepThresholdMps2;
            enoughTime = ...
                (sampleTime - lastHitTime) >= cfg.MinInterHitSeconds;

            if changedToUpward && bottomJerk && enoughTime
                play(players{nextVoice});
                nextVoice = mod(nextVoice, numel(players)) + 1;
                hitCount = hitCount + 1;
                fprintf('KICK %d   reversal=%+.2f rad/s   accel step=%.2f m/s^2\n', ...
                    hitCount, signedRate, max(recentAccelSteps));

                lastHitTime = sampleTime;
                returnTravelRad = 0;
                state = WAIT_RETURN_TRAVEL;

            elseif signedRate <= cfg.MissedHitResetRadPerSec
                state = WAIT_DOWNSTROKE;
            end

        else % WAIT_RETURN_TRAVEL
            returnTravelRad = returnTravelRad + ...
                max(0, -signedRate) * samplePeriod;
            enoughTravel = ...
                returnTravelRad >= cfg.MinReturnTravelRad;
            enoughTime = ...
                (sampleTime - lastHitTime) >= cfg.MinInterHitSeconds;
            if enoughTravel && enoughTime
                state = WAIT_DOWNSTROKE;
            end
        end
    end
end

function imu = setupMPU6050(board, sampleRate)
    divider = 1000 / sampleRate - 1;
    if divider < 0 || divider > 255 || divider ~= round(divider)
        error('SensorOutputRate must produce an integer MPU6050 divider.');
    end

    imu = device(board, 'I2CAddress', '0x68');
    whoAmI = readRegister(imu, hex2dec('75'), 1, 'uint8');
    if whoAmI ~= hex2dec('68')
        error('Device at 0x68 is not responding as an MPU6050.');
    end

    % Original working ranges: gyro +/-250 dps, accelerometer +/-2 g.
    writeRegister(imu, hex2dec('6B'), hex2dec('01'), 'uint8');
    pause(0.10);
    writeRegister(imu, hex2dec('1A'), hex2dec('04'), 'uint8');
    writeRegister(imu, hex2dec('19'), uint8(divider), 'uint8');
    writeRegister(imu, hex2dec('1B'), hex2dec('00'), 'uint8');
    writeRegister(imu, hex2dec('1C'), hex2dec('00'), 'uint8');
    pause(0.10);
end

function sampleRate = measureReadRate(imu, sampleCount)
    for k = 1:3
        readMPU6050(imu);
    end
    timer = tic;
    for k = 1:sampleCount
        readMPU6050(imu);
    end
    sampleRate = sampleCount / toc(timer);
    if ~isfinite(sampleRate) || sampleRate < 5
        error('Invalid MATLAB IMU read rate: %.2f Hz.', sampleRate);
    end
    fprintf('Measured MATLAB IMU read rate: %.2f Hz.\n', sampleRate);
end

function gyroBiasDps = measureGyroBias(imu, sampleRate, durationSeconds)
    fprintf('\nKeep the stick still. Gyro calibration starts in 2 seconds...\n');
    for k = 1:max(1, round(2 * sampleRate))
        readMPU6050(imu);
    end
    fprintf('Calibrating now; keep still...\n');

    count = max(10, round(sampleRate * durationSeconds));
    gyro = zeros(count, 3);
    for k = 1:count
        sample = readMPU6050(imu);
        gyro(k, :) = sample.gyroDps;
    end

    gyroBiasDps = median(gyro, 1);
    robustNoise = 1.4826 * median(abs(gyro - gyroBiasDps), 1);
    fprintf('Gyro bias [X Y Z]:  %+.3f %+.3f %+.3f dps\n', ...
        gyroBiasDps(1), gyroBiasDps(2), gyroBiasDps(3));
    fprintf('Gyro noise [X Y Z]: %.3f %.3f %.3f dps\n', ...
        robustNoise(1), robustNoise(2), robustNoise(3));
    if any(robustNoise > 5)
        error('The stick moved substantially during gyro calibration.');
    end
end

function direction = learnDownstrokeDirection(imu, gyroBiasDps, ...
        sampleRate, durationSeconds, minimumPeakRate, saveFile)
    fprintf('\nONE-STROKE DIRECTION SETUP\n');
    fprintf('Hold the stick at the top and wait for GO.\n');
    for secondsRemaining = 3:-1:1
        fprintf('%d...\n', secondsRemaining);
        for k = 1:max(1, round(sampleRate))
            readMPU6050(imu);
        end
    end

    fprintf('GO: make one normal downward stroke now.\n');
    count = max(20, round(sampleRate * durationSeconds));
    gyro = zeros(count, 3);
    for k = 1:count
        sample = readMPU6050(imu);
        gyro(k, :) = deg2rad(sample.gyroDps - gyroBiasDps);
    end

    peakByAxis = max(abs(gyro), [], 1);
    fprintf('Stroke peaks [X Y Z]: %.3f %.3f %.3f rad/s\n', ...
        peakByAxis(1), peakByAxis(2), peakByAxis(3));

    [~, axisIndex] = max(sum(gyro.^2, 1));
    axisSignal = gyro(:, axisIndex);
    peakRate = max(abs(axisSignal));
    if peakRate < minimumPeakRate
        error(['No clear rotation was measured (peak %.3f rad/s). Make the ' ...
            'downstroke as a normal drumstick swing that pivots at the wrist.'], ...
            peakRate);
    end

    strong = abs(axisSignal) >= 0.65 * peakRate;
    signedMotion = median(axisSignal(strong));
    if abs(signedMotion) < 0.05
        [~, peakIndex] = max(abs(axisSignal));
        signedMotion = axisSignal(peakIndex);
    end

    axisNames = ["X", "Y", "Z"];
    direction.axis = axisIndex;
    direction.axisName = axisNames(axisIndex);
    direction.sign = sign(signedMotion);
    validateDirection(direction);
    save(saveFile, 'direction');
    fprintf('Learned downward direction: %s gyro, sign %+d.\n', ...
        direction.axisName, direction.sign);
end

function validateDirection(direction)
    if ~isfield(direction, 'axis') || ...
            ~isfield(direction, 'axisName') || ...
            ~isfield(direction, 'sign') || ...
            ~ismember(direction.axis, 1:3) || ...
            ~ismember(direction.sign, [-1 1])
        error(['Direction calibration is invalid. Delete ' ...
            'kickDirectionCalibration.mat and run again.']);
    end
end

function sample = readMPU6050(imu)
    maxAttempts = 30;
    raw = uint8([]);
    for attempt = 1:maxAttempts
        raw = readRegister(imu, hex2dec('3B'), 14, 'uint8');
        raw = raw(:).';
        if numel(raw) == 14 && any(raw ~= 0)
            break;
        end

        if attempt == 5 || attempt == 15
            try
                writeRegister(imu, hex2dec('6B'), hex2dec('01'), 'uint8');
                pause(0.10);
                writeRegister(imu, hex2dec('1A'), hex2dec('04'), 'uint8');
                writeRegister(imu, hex2dec('1B'), hex2dec('00'), 'uint8');
                writeRegister(imu, hex2dec('1C'), hex2dec('00'), 'uint8');
            catch
                % Continue retrying a brief I2C interruption.
            end
        end
        pause(0.02);
    end

    if numel(raw) ~= 14 || all(raw == 0)
        error(['MPU6050 stayed unavailable after %d retries. Check power, ' ...
            'ground, SDA and SCL, then run clear and restart.'], maxAttempts);
    end

    sample.accelG = [ ...
        signed16(raw(1), raw(2)), ...
        signed16(raw(3), raw(4)), ...
        signed16(raw(5), raw(6))] / 16384;
    sample.gyroDps = [ ...
        signed16(raw(9), raw(10)), ...
        signed16(raw(11), raw(12)), ...
        signed16(raw(13), raw(14))] / 131;
end

function value = signed16(highByte, lowByte)
    bits = bitor(bitshift(uint16(highByte), 8), uint16(lowByte));
    if bits >= 32768
        value = double(bits) - 65536;
    else
        value = double(bits);
    end
end

function players = createKickPlayers(numberOfVoices)
    audioFs = 22050;
    t = (0:round(0.28 * audioFs)-1)' / audioFs;
    phase = 2*pi*(58*t + 62*(1-exp(-22*t))/22);
    kick = sin(phase) .* exp(-15*t);
    click = 0.18 * randn(size(t)) .* exp(-75*t);
    kick = kick + click;
    kick = 0.90 * kick / max(abs(kick));

    players = cell(1, numberOfVoices);
    for k = 1:numberOfVoices
        players{k} = audioplayer(kick, audioFs); %#ok<TNMLP>
    end
end

function shutdownAll(players)
    for k = 1:numel(players)
        try
            stop(players{k});
        catch
            % Player may already be released.
        end
    end
    fprintf('\nMyoDrum stopped.\n');
end
