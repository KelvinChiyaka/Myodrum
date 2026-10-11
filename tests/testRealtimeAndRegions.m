function tests = testRealtimeAndRegions
% TESTREALTIMEANDREGIONS Hardware-free tests for the live three-drum path.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'calibration'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    testCase.TestData.Root = root;
end

function testDecodeFrameSignedBoundaries(testCase)
    raw = uint8([64 0, 192 0, 127 255, 0 0, ...
        0 131, 255 125, 128 0]);
    sample = decodeMPU6050Frame(raw);
    verifyEqual(testCase, sample.accelG(1:2), [1, -1], 'AbsTol', 1e-12);
    verifyEqual(testCase, sample.accelG(3), 32767 / 16384, ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, sample.temperatureC, 36.53, 'AbsTol', 1e-12);
    verifyEqual(testCase, sample.gyroDps(1:2), [1, -1], ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, sample.gyroDps(3), -32768 / 131, ...
        'AbsTol', 1e-12);
end

function testOrientationWindowUsesEverySample(testCase)
    window = OrientationWindowAverager(4);
    for sampleIndex = 1:3
        [ready, ~, ~] = window.add(sampleIndex * [1 2 3], ...
            sampleIndex * [4 5 6]);
        verifyFalse(testCase, ready);
    end
    [ready, accel, gyro] = window.add(4 * [1 2 3], 4 * [4 5 6]);
    verifyTrue(testCase, ready);
    verifyEqual(testCase, accel, 2.5 * [1 2 3], 'AbsTol', 1e-12);
    verifyEqual(testCase, gyro, 2.5 * [4 5 6], 'AbsTol', 1e-12);
end

function testFreshGyroPolicyIgnoresCache(testCase)
    folder = string(tempname);
    mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder, 's'));
    cacheFile = fullfile(folder, 'gyro.mat');
    saveGyroCache(cacheFile, ones(2, 3), zeros(2, 3), datetime('now'));
    [bias, ~, source] = resolveGyroBias(@fakeGyroMeasurement, ...
        cacheFile, "fresh", 8, false);
    verifyEqual(testCase, bias, 2 * ones(2, 3));
    verifyEqual(testCase, source, "measured");
end

function testCachedGyroPolicyUsesYoungCache(testCase)
    folder = string(tempname);
    mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder, 's'));
    cacheFile = fullfile(folder, 'gyro.mat');
    saveGyroCache(cacheFile, ones(2, 3), zeros(2, 3), datetime('now'));
    [bias, ~, source] = resolveGyroBias(@sentinelGyroMeasurement, ...
        cacheFile, "cached", 8, false);
    verifyEqual(testCase, bias, ones(2, 3));
    verifyEqual(testCase, source, "cached");
end

function testArmTimeoutReturnsToIdle(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    direction = struct('axis', 3, 'sign', 1);
    detector = initializeKickDetector;
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0.8], direction, 0, 20, cfg.Kick);
    [detector, hit] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0.3], direction, 1.01, 20, cfg.Kick);
    verifyFalse(testCase, hit);
    verifyEqual(testCase, detector.mode, 1);
end

function testHandSpecificRegionClassification(testCase)
    targetNames = ["KICK", "SNARE", "CRASH"];
    angles = [40 0; 42 1; -40 0; -42 -1; 0 25; 1 24; ...
        50 5; 52 4; -30 5; -32 6; 10 30; 11 29];
    hands = [ones(6, 1); 2 * ones(6, 1)];
    labels = repmat(["KICK"; "KICK"; "SNARE"; "SNARE"; ...
        "CRASH"; "CRASH"], 2, 1);
    model = buildDrumRegionModel(angles, hands, labels, targetNames, [5 5]);
    [leftLabel, ~] = classifyDrumRegion(model, 1, [-41 0], 3, 0.35);
    [rightLabel, ~] = classifyDrumRegion(model, 2, [51 5], 3, 0.35);
    verifyEqual(testCase, leftLabel, "SNARE");
    verifyEqual(testCase, rightLabel, "KICK");
end

function testEitherHandCanSelectEveryDrum(testCase)
    targetNames = ["KICK", "SNARE", "CRASH"];
    base = [35 0; 36 1; -35 0; -36 -1; 0 35; 1 36];
    angles = [base; base];
    hands = [ones(6, 1); 2 * ones(6, 1)];
    labels = repmat(["KICK"; "KICK"; "SNARE"; "SNARE"; ...
        "CRASH"; "CRASH"], 2, 1);
    model = buildDrumRegionModel(angles, hands, labels, targetNames);
    for hand = 1:2
        verifyEqual(testCase, classifyDrumRegion( ...
            model, hand, [35 0]), "KICK");
        verifyEqual(testCase, classifyDrumRegion( ...
            model, hand, [-35 0]), "SNARE");
        verifyEqual(testCase, classifyDrumRegion( ...
            model, hand, [0 35]), "CRASH");
    end
end

function testRegionHysteresisAndNeutralRejection(testCase)
    targetNames = ["KICK", "SNARE", "CRASH"];
    angles = [35 0; 36 1; -35 0; -36 -1; 0 35; 1 36; ...
        35 0; 36 1; -35 0; -36 -1; 0 35; 1 36];
    hands = [ones(6, 1); 2 * ones(6, 1)];
    labels = repmat(["KICK"; "KICK"; "SNARE"; "SNARE"; ...
        "CRASH"; "CRASH"], 2, 1);
    model = buildDrumRegionModel(angles, hands, labels, targetNames);
    retained = classifyDrumRegion(model, 1, [18 0], 3, 0.35, ...
        "KICK", 3.75);
    switched = classifyDrumRegion(model, 1, [-35 0], 3, 0.35, ...
        "KICK", 3.75);
    rejected = classifyDrumRegion(model, 1, [80 100], 3, 0.35);
    verifyEqual(testCase, retained, "KICK");
    verifyEqual(testCase, switched, "SNARE");
    verifyEqual(testCase, rejected, "");
end

function [bias, noise] = fakeGyroMeasurement
    bias = 2 * ones(2, 3);
    noise = 0.1 * ones(2, 3);
end

function [bias, noise] = sentinelGyroMeasurement
    bias = 9 * ones(2, 3);
    noise = 9 * ones(2, 3);
end

function saveGyroCache(file, bias, noise, createdAt)
    gyroCalibration.biasDps = bias;
    gyroCalibration.noiseDps = noise;
    gyroCalibration.createdAt = string(createdAt);
    save(file, 'gyroCalibration');
end
