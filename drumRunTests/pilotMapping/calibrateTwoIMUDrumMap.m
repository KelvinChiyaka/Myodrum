function calibrateTwoIMUDrumMap
% CALIBRATETWOIMUDRUMMAP Learn five drum zones from two MPU6050 sensors.
% IMU 1: address 0x68. IMU 2: address 0x69 (AD0 held high).
%
% Each drum is collected in five blocks of 20 accepted hits per hand. Every
% hit stores a short pre-hit motion window rather than attempting unreliable
% double integration. Separate nearest-centroid maps are trained for each hand.

    clc;

    %% USER SETTINGS
    cfg.COMPort = "COM7";
    cfg.Board = "Mega2560";
    cfg.Addresses = ["0x68", "0x69"];
    cfg.HandNames = ["LEFT", "RIGHT"]; % swap if your addresses are reversed
    cfg.SensorOutputRate = 50;
    cfg.Gravity = 9.80665;

    cfg.DrumNames = ["CRASH", "HIHAT", "TOM", "KICK", "SNARE"];
    cfg.Rounds = 5;
    cfg.HitsPerBlockPerIMU = 20;
    cfg.SwitchCountdownSeconds = 4;

    cfg.DownstrokeArmRadPerSec = 0.50;
    cfg.BottomReversalRadPerSec = -0.10;
    cfg.AccelStepThresholdMps2 = 2.5;
    cfg.MissedHitResetRadPerSec = -0.60;
    cfg.MinReturnTravelRad = deg2rad(8);
    cfg.MinInterHitSeconds = 0.08;
    cfg.FeatureWindowSeconds = 0.55;
    cfg.DirectionMinimumPeakRadPerSec = 0.30;

    thisFolder = fileparts(mfilename('fullpath'));
    directionFile = fullfile(thisFolder, 'twoIMUDirectionCalibration.mat');
    outputMat = fullfile(thisFolder, 'twoIMUDrumMap.mat');
    outputCsv = fullfile(thisFolder, 'twoIMUDrumHits.csv');

    %% CONNECT BOTH IMUS
    fprintf('MyoDrum: two-IMU five-drum mapping\n');
    fprintf('Connecting to %s on %s...\n', cfg.Board, cfg.COMPort);
    board = arduino(cfg.COMPort, cfg.Board, 'Libraries', 'I2C');
    sensors(1) = setupMPU6050(board, cfg.Addresses(1), cfg.SensorOutputRate);
    sensors(2) = setupMPU6050(board, cfg.Addresses(2), cfg.SensorOutputRate);
    sampleRate = measureCombinedReadRate(sensors, 20);

    %% CALIBRATE EACH HAND
    gyroBiasDps = measureTwoGyroBiases(sensors, sampleRate, 1.5);
    if isfile(directionFile)
        loaded = load(directionFile, 'directions');
        directions = loaded.directions;
        validateDirections(directions);
        fprintf('Loaded saved downstroke directions for both IMUs.\n');
    else
        directions = repmat(struct('axis', 0, 'axisName', "", 'sign', 0), 1, 2);
        for sensorIndex = 1:2
            fprintf('\nDirection setup for %s hand (%s). Use only that stick.\n', ...
                cfg.HandNames(sensorIndex), cfg.Addresses(sensorIndex));
            directions(sensorIndex) = learnDirection( ...
                sensors, sensorIndex, gyroBiasDps(sensorIndex, :), ...
                sampleRate, cfg.DirectionMinimumPeakRadPerSec);
        end
        save(directionFile, 'directions');
    end

    %% STORAGE
    numberOfDrums = numel(cfg.DrumNames);
    maximumRecords = 2 * numberOfDrums * cfg.Rounds * ...
        cfg.HitsPerBlockPerIMU;
    featureCount = 15;
    features = zeros(maximumRecords, featureCount);
    drumLabel = strings(maximumRecords, 1);
    handLabel = strings(maximumRecords, 1);
    imuAddress = strings(maximumRecords, 1);
    roundNumber = zeros(maximumRecords, 1);
    blockHitNumber = zeros(maximumRecords, 1);
    elapsedTime = zeros(maximumRecords, 1);
    recordCount = 0;

    confirmationPlayers = createConfirmationPlayers;
    nextVoice = ones(1, numberOfDrums);
    cleanup = onCleanup(@()shutdownAll(confirmationPlayers));

    fprintf('\nMAPPING COLLECTION\n');
    fprintf('For every target, alternate hands naturally until BOTH counters reach %d.\n', ...
        cfg.HitsPerBlockPerIMU);
    fprintf('Use the same comfortable home pose between strikes. Ctrl+C stops safely.\n');

    sessionTimer = tic;

    %% FIVE ROUNDS; ROTATE ORDER TO REDUCE TIME/DRIFT BIAS
    for roundIndex = 1:cfg.Rounds
        order = circshift(1:numberOfDrums, -(roundIndex - 1));
        fprintf('\n========== ROUND %d OF %d ==========\n', ...
            roundIndex, cfg.Rounds);

        for orderIndex = 1:numberOfDrums
            drumIndex = order(orderIndex);
            currentDrum = cfg.DrumNames(drumIndex);

            fprintf('\nNEXT TARGET: %s\n', currentDrum);
            fprintf('Layout reminder: CRASH far-left | HIHAT rear-centre | ');
            fprintf('TOM rear-right | KICK front-centre | SNARE front-right\n');
            keepSensorsAliveCountdown( ...
                sensors, cfg.SwitchCountdownSeconds);
            fprintf('GO: strike %s until both hands reach %d accepted hits.\n', ...
                currentDrum, cfg.HitsPerBlockPerIMU);

            detectors(1) = initializeDetector(sampleRate, cfg);
            detectors(2) = initializeDetector(sampleRate, cfg);
            blockCounts = zeros(1, 2);

            while any(blockCounts < cfg.HitsPerBlockPerIMU)
                for sensorIndex = 1:2
                    sample = readMPU6050(sensors(sensorIndex));
                    correctedGyroRad = deg2rad( ...
                        sample.gyroDps - gyroBiasDps(sensorIndex, :));
                    now = toc(sessionTimer);

                    [detectors(sensorIndex), hit, hitFeature] = ...
                        updateDetector(detectors(sensorIndex), sample.accelG, ...
                        correctedGyroRad, directions(sensorIndex), now, ...
                        sampleRate, cfg);

                    if hit && blockCounts(sensorIndex) < cfg.HitsPerBlockPerIMU
                        blockCounts(sensorIndex) = blockCounts(sensorIndex) + 1;
                        recordCount = recordCount + 1;
                        features(recordCount, :) = hitFeature;
                        drumLabel(recordCount) = currentDrum;
                        handLabel(recordCount) = cfg.HandNames(sensorIndex);
                        imuAddress(recordCount) = cfg.Addresses(sensorIndex);
                        roundNumber(recordCount) = roundIndex;
                        blockHitNumber(recordCount) = blockCounts(sensorIndex);
                        elapsedTime(recordCount) = now;

                        voices = confirmationPlayers.(char(currentDrum));
                        play(voices{nextVoice(drumIndex)});
                        nextVoice(drumIndex) = ...
                            mod(nextVoice(drumIndex), numel(voices)) + 1;

                        fprintf('%-6s  %s %3d/%d   %s %3d/%d\n', ...
                            currentDrum, cfg.HandNames(1), blockCounts(1), ...
                            cfg.HitsPerBlockPerIMU, cfg.HandNames(2), ...
                            blockCounts(2), cfg.HitsPerBlockPerIMU);
                    end
                end
            end
        end
    end

    %% BUILD TABLE AND SEPARATE MAP FOR EACH HAND
    features = features(1:recordCount, :);
    drumLabel = drumLabel(1:recordCount);
    handLabel = handLabel(1:recordCount);
    imuAddress = imuAddress(1:recordCount);
    roundNumber = roundNumber(1:recordCount);
    blockHitNumber = blockHitNumber(1:recordCount);
    elapsedTime = elapsedTime(1:recordCount);

    metadata = table(elapsedTime, handLabel, imuAddress, drumLabel, ...
        roundNumber, blockHitNumber, 'VariableNames', ...
        {'Time_s', 'Hand', 'IMUAddress', 'Drum', 'Round', 'HitInBlock'});
    featureNames = cellstr(compose('F%02d', 1:featureCount));
    featureTable = array2table(features, 'VariableNames', featureNames);
    hitTable = [metadata, featureTable];

    models = trainHandModels(features, drumLabel, handLabel, ...
        roundNumber, cfg.HandNames, cfg.DrumNames);
    save(outputMat, 'hitTable', 'models', 'cfg', 'sampleRate', ...
        'gyroBiasDps', 'directions');
    writetable(hitTable, outputCsv);

    fprintf('\nMapping complete: %d labelled hits saved.\n', recordCount);
    for sensorIndex = 1:2
        fprintf('%s cross-round validation accuracy: %.1f%%\n', ...
            cfg.HandNames(sensorIndex), ...
            100 * models(sensorIndex).validationAccuracy);
    end
    fprintf('Saved model:\n  %s\n', outputMat);
    fprintf('Saved hit table:\n  %s\n', outputCsv);
end

function sensor = setupMPU6050(board, address, outputRate)
    divider = 1000 / outputRate - 1;
    if divider < 0 || divider > 255 || divider ~= round(divider)
        error('SensorOutputRate must produce an integer divider.');
    end
    sensor.dev = device(board, 'I2CAddress', char(address));
    sensor.address = address;
    id = readRegister(sensor.dev, hex2dec('75'), 1, 'uint8');
    if id ~= hex2dec('68')
        error('No MPU6050 responding correctly at %s.', address);
    end
    sensor.initRegs = [107, 1; 26, 4; 25, divider; 27, 0; 28, 0];
    applyConfig(sensor);
    fprintf('Connected MPU6050 at %s.\n', address);
end

function applyConfig(sensor)
    for k = 1:size(sensor.initRegs, 1)
        writeRegister(sensor.dev, sensor.initRegs(k, 1), ...
            uint8(sensor.initRegs(k, 2)), 'uint8');
        if k == 1, pause(0.10); end
    end
    pause(0.05);
end

function sampleRate = measureCombinedReadRate(sensors, loopCount)
    for k = 1:3
        readMPU6050(sensors(1));
        readMPU6050(sensors(2));
    end
    timer = tic;
    for k = 1:loopCount
        readMPU6050(sensors(1));
        readMPU6050(sensors(2));
    end
    sampleRate = loopCount / toc(timer);
    if sampleRate < 5
        error('Combined per-IMU read rate is too low: %.2f Hz.', sampleRate);
    end
    fprintf('Measured read rate per IMU: %.2f Hz.\n', sampleRate);
end

function biases = measureTwoGyroBiases(sensors, sampleRate, seconds)
    fprintf('\nKeep BOTH sticks still. Calibration begins in 2 seconds...\n');
    for k = 1:max(1, round(2 * sampleRate))
        readMPU6050(sensors(1));
        readMPU6050(sensors(2));
    end
    count = max(10, round(sampleRate * seconds));
    gyro = zeros(count, 3, 2);
    for k = 1:count
        for sensorIndex = 1:2
            sample = readMPU6050(sensors(sensorIndex));
            gyro(k, :, sensorIndex) = sample.gyroDps;
        end
    end
    biases = zeros(2, 3);
    for sensorIndex = 1:2
        values = gyro(:, :, sensorIndex);
        biases(sensorIndex, :) = median(values, 1);
        noise = 1.4826 * median(abs(values - biases(sensorIndex, :)), 1);
        fprintf('IMU %d bias [%+.2f %+.2f %+.2f], noise [%.2f %.2f %.2f] dps\n', ...
            sensorIndex, biases(sensorIndex, 1), biases(sensorIndex, 2), ...
            biases(sensorIndex, 3), noise(1), noise(2), noise(3));
        if any(noise > 5)
            error('IMU %d moved substantially during gyro calibration.', sensorIndex);
        end
    end
end

function direction = learnDirection(sensors, targetIndex, bias, sampleRate, minimumPeak)
    for secondsRemaining = 3:-1:1
        fprintf('%d...\n', secondsRemaining);
        for k = 1:max(1, round(sampleRate))
            readMPU6050(sensors(1));
            readMPU6050(sensors(2));
        end
    end
    fprintf('GO: make one normal downward wrist stroke with IMU %d.\n', targetIndex);
    count = max(20, round(2.5 * sampleRate));
    gyro = zeros(count, 3);
    for k = 1:count
        for sensorIndex = 1:2
            sample = readMPU6050(sensors(sensorIndex));
            if sensorIndex == targetIndex
                gyro(k, :) = deg2rad(sample.gyroDps - bias);
            end
        end
    end
    peaks = max(abs(gyro), [], 1);
    fprintf('IMU %d stroke peaks [X Y Z]: %.2f %.2f %.2f rad/s\n', ...
        targetIndex, peaks(1), peaks(2), peaks(3));
    [~, axisIndex] = max(sum(gyro.^2, 1));
    axisSignal = gyro(:, axisIndex);
    peak = max(abs(axisSignal));
    if peak < minimumPeak
        error('IMU %d did not measure a clear wrist stroke.', targetIndex);
    end
    strong = abs(axisSignal) >= 0.65 * peak;
    signedMotion = median(axisSignal(strong));
    if abs(signedMotion) < 0.05
        [~, index] = max(abs(axisSignal));
        signedMotion = axisSignal(index);
    end
    names = ["X", "Y", "Z"];
    direction.axis = axisIndex;
    direction.axisName = names(axisIndex);
    direction.sign = sign(signedMotion);
end

function validateDirections(directions)
    if numel(directions) ~= 2
        error('Saved direction calibration is invalid. Delete it and rerun.');
    end
    for k = 1:2
        if ~isfield(directions(k), 'axis') || ...
                ~isfield(directions(k), 'sign') || ...
                ~ismember(directions(k).axis, 1:3) || ...
                ~ismember(directions(k).sign, [-1 1])
            error('Saved direction calibration is invalid. Delete it and rerun.');
        end
    end
end

function detector = initializeDetector(sampleRate, cfg)
    detector.mode = 1; % 1 wait down, 2 wait bottom, 3 wait return
    detector.previousAccel = [];
    detector.accelSteps = zeros(1, 3);
    detector.returnTravel = 0;
    detector.lastHitTime = -inf;
    detector.lastSampleTime = NaN;
    detector.bufferLength = max(5, round(cfg.FeatureWindowSeconds * sampleRate));
    detector.accelBuffer = zeros(detector.bufferLength, 3);
    detector.gyroBuffer = zeros(detector.bufferLength, 3);
    detector.bufferCount = 0;
end

function [detector, hit, feature] = updateDetector(detector, accelG, gyroRad, ...
        direction, timeNow, sampleRate, cfg)
    hit = false;
    feature = zeros(1, 15);
    signedRate = direction.sign * gyroRad(direction.axis);
    accelMps2 = accelG * cfg.Gravity;

    if isempty(detector.previousAccel)
        step = 0;
    else
        step = norm(accelMps2 - detector.previousAccel);
    end
    detector.previousAccel = accelMps2;
    detector.accelSteps = [detector.accelSteps(2:3), step];

    detector.accelBuffer = [detector.accelBuffer(2:end, :); accelG];
    detector.gyroBuffer = [detector.gyroBuffer(2:end, :); gyroRad];
    detector.bufferCount = min(detector.bufferCount + 1, detector.bufferLength);

    if isnan(detector.lastSampleTime)
        period = 1 / sampleRate;
    else
        period = min(max(timeNow - detector.lastSampleTime, 0), 2 / sampleRate);
    end
    detector.lastSampleTime = timeNow;

    if detector.mode == 1
        if signedRate >= cfg.DownstrokeArmRadPerSec
            detector.mode = 2;
            detector.accelSteps(:) = 0;
        end
    elseif detector.mode == 2
        reversal = signedRate <= cfg.BottomReversalRadPerSec;
        jerk = max(detector.accelSteps) >= cfg.AccelStepThresholdMps2;
        gap = (timeNow - detector.lastHitTime) >= cfg.MinInterHitSeconds;
        if reversal && jerk && gap
            hit = true;
            feature = makeFeature(detector, period);
            detector.lastHitTime = timeNow;
            detector.returnTravel = 0;
            detector.mode = 3;
        elseif signedRate <= cfg.MissedHitResetRadPerSec
            detector.mode = 1;
        end
    else
        detector.returnTravel = detector.returnTravel + max(0, -signedRate) * period;
        if detector.returnTravel >= cfg.MinReturnTravelRad && ...
                (timeNow - detector.lastHitTime) >= cfg.MinInterHitSeconds
            detector.mode = 1;
        end
    end
end

function feature = makeFeature(detector, period)
    validStart = detector.bufferLength - detector.bufferCount + 1;
    accel = detector.accelBuffer(validStart:end, :);
    gyro = detector.gyroBuffer(validStart:end, :);
    earlyCount = max(1, floor(size(accel, 1) / 2));
    earlyAccel = mean(accel(1:earlyCount, :), 1);
    meanAccel = mean(accel, 1);
    gyroIntegral = sum(gyro, 1) * period;
    peakGyro = max(abs(gyro), [], 1);
    finalGyro = gyro(end, :);
    feature = [earlyAccel, meanAccel, gyroIntegral, peakGyro, finalGyro];
end

function models = trainHandModels(features, labels, hands, rounds, handNames, drumNames)
    models = repmat(struct, 1, 2);
    for handIndex = 1:2
        selected = hands == handNames(handIndex);
        x = features(selected, :);
        y = labels(selected);
        r = rounds(selected);
        mu = mean(x, 1);
        sigma = std(x, 0, 1);
        sigma(sigma < 1e-9) = 1;
        z = (x - mu) ./ sigma;
        centroids = calculateCentroids(z, y, drumNames);

        predicted = strings(size(y));
        for fold = 1:max(r)
            train = r ~= fold;
            test = r == fold;
            foldCentroids = calculateCentroids(z(train, :), y(train), drumNames);
            predicted(test) = classifyByCentroid(z(test, :), foldCentroids, drumNames);
        end

        models(handIndex).hand = handNames(handIndex);
        models(handIndex).featureMean = mu;
        models(handIndex).featureStd = sigma;
        models(handIndex).centroids = centroids;
        models(handIndex).drumNames = drumNames;
        models(handIndex).validationAccuracy = mean(predicted == y);
        models(handIndex).confusion = makeConfusion(y, predicted, drumNames);
    end
end

function centroids = calculateCentroids(x, labels, drumNames)
    centroids = zeros(numel(drumNames), size(x, 2));
    for k = 1:numel(drumNames)
        rows = labels == drumNames(k);
        if ~any(rows)
            error('No training hits were recorded for %s.', drumNames(k));
        end
        centroids(k, :) = mean(x(rows, :), 1);
    end
end

function predicted = classifyByCentroid(x, centroids, drumNames)
    predicted = strings(size(x, 1), 1);
    for row = 1:size(x, 1)
        distance = sum((centroids - x(row, :)).^2, 2);
        [~, index] = min(distance);
        predicted(row) = drumNames(index);
    end
end

function matrix = makeConfusion(actual, predicted, drumNames)
    matrix = zeros(numel(drumNames), numel(drumNames));
    for row = 1:numel(actual)
        actualIndex = find(drumNames == actual(row), 1);
        predictedIndex = find(drumNames == predicted(row), 1);
        matrix(actualIndex, predictedIndex) = ...
            matrix(actualIndex, predictedIndex) + 1;
    end
end

function keepSensorsAliveCountdown(sensors, seconds)
    for remaining = seconds:-1:1
        fprintf('%d...\n', remaining);
        timer = tic;
        while toc(timer) < 1
            readMPU6050(sensors(1));
            readMPU6050(sensors(2));
        end
    end
end

function sample = readMPU6050(sensor)
    raw = uint8([]);
    for attempt = 1:20
        raw = readRegister(sensor.dev, hex2dec('3B'), 14, 'uint8');
        raw = raw(:).';
        if numel(raw) == 14 && any(raw ~= 0)
            break;
        end
        if attempt == 5 || attempt == 12
            try
                applyConfig(sensor);
            catch
                % Continue retrying after a brief I2C interruption.
            end
        end
        pause(0.02);
    end
    if numel(raw) ~= 14 || all(raw == 0)
        error('Invalid data from MPU6050 at %s. Check its wiring.', sensor.address);
    end
    sample.accelG = [signed16(raw(1),raw(2)), signed16(raw(3),raw(4)), ...
        signed16(raw(5),raw(6))] / 16384;
    sample.gyroDps = [signed16(raw(9),raw(10)), signed16(raw(11),raw(12)), ...
        signed16(raw(13),raw(14))] / 131;
end

function value = signed16(highByte, lowByte)
    bits = bitor(bitshift(uint16(highByte), 8), uint16(lowByte));
    value = double(bits);
    if bits >= 32768, value = value - 65536; end
end

function players = createConfirmationPlayers
    names = ["CRASH", "HIHAT", "TOM", "KICK", "SNARE"];
    frequencies = [900, 1200, 150, 65, 220];
    fs = 22050;
    for drumIndex = 1:numel(names)
        t = (0:round(0.12 * fs)-1)' / fs;
        tone = sin(2*pi*frequencies(drumIndex)*t) .* exp(-25*t);
        tone = 0.65 * tone / max(abs(tone));
        field = char(names(drumIndex));
        players.(field) = cell(1, 3);
        for voice = 1:3
            players.(field){voice} = audioplayer(tone, fs); %#ok<TNMLP>
        end
    end
end

function shutdownAll(players)
    fields = fieldnames(players);
    for fieldIndex = 1:numel(fields)
        voices = players.(fields{fieldIndex});
        for voice = 1:numel(voices)
            try, stop(voices{voice}); catch, end %#ok<NOCOM>
        end
    end
    fprintf('\nTwo-IMU mapping stopped.\n');
end
