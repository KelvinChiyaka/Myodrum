function tests = testProjectStructure
% TESTPROJECTSTRUCTURE Hardware-independent checks for the repository layout.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    testCase.TestData.Root = root;
end

function testTwoHandsCanTriggerInSameCycle(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    direction = struct('axis', 3, 'sign', 1);
    detectors = [initializeKickDetector, initializeKickDetector];
    hit = false(1, 2);
    for sensorIndex = 1:2
        [detectors(sensorIndex), ~] = updateKickDetector( ...
            detectors(sensorIndex), [0 0 9.81], [0 0 0], ...
            direction, 0.00, 20, cfg.Kick);
        [detectors(sensorIndex), ~] = updateKickDetector( ...
            detectors(sensorIndex), [0 0 9.81], [0 0 0.8], ...
            direction, 0.05, 20, cfg.Kick);
        [detectors(sensorIndex), hit(sensorIndex)] = updateKickDetector( ...
            detectors(sensorIndex), [4 0 9.81], [0 0 0.2], ...
            direction, 0.10, 20, cfg.Kick);
    end
    verifyTrue(testCase, all(hit));
end

function testReturnTravelRearmsNextStroke(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    direction = struct('axis', 3, 'sign', 1);
    detector = initializeKickDetector;
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0], direction, 0.00, 20, cfg.Kick);
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0.8], direction, 0.05, 20, cfg.Kick);
    [detector, firstHit] = updateKickDetector(detector, [4 0 9.81], ...
        [0 0 0.2], direction, 0.10, 20, cfg.Kick);
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 -2.0], direction, 0.20, 20, cfg.Kick);
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 -1.5], direction, 0.28, 20, cfg.Kick);
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0.8], direction, 0.40, 20, cfg.Kick);
    [~, secondHit] = updateKickDetector(detector, [4 0 9.81], ...
        [0 0 0.2], direction, 0.45, 20, cfg.Kick);
    verifyTrue(testCase, firstHit);
    verifyTrue(testCase, secondHit);
end


function testReboundInsideGuardDoesNotRetrigger(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    direction = struct('axis', 3, 'sign', 1);
    detector = initializeKickDetector;
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0], direction, 0.00, 20, cfg.Kick);
    [detector, ~] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0.8], direction, 0.05, 20, cfg.Kick);
    [detector, firstHit] = updateKickDetector(detector, [4 0 9.81], ...
        [0 0 0.2], direction, 0.10, 20, cfg.Kick);
    [detector, reboundHit1] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 -3.0], direction, 0.20, 20, cfg.Kick);
    [detector, reboundHit2] = updateKickDetector(detector, [0 0 9.81], ...
        [0 0 0.9], direction, 0.25, 20, cfg.Kick);
    [~, reboundHit3] = updateKickDetector(detector, [4 0 9.81], ...
        [0 0 0.1], direction, 0.30, 20, cfg.Kick);
    verifyTrue(testCase, firstHit);
    verifyFalse(testCase, reboundHit1 || reboundHit2 || reboundHit3);
end

function testRequiredFoldersExist(testCase)
    required = { ...
        'IMU_subsystem', 'EMG_subsystem', 'calibration', ...
        fullfile('classifiers', 'ruleBased'), ...
        fullfile('classifiers', 'machineLearning'), ...
        'drumRunTests', 'common', 'data', 'audio', 'tests', 'docs'};
    for k = 1:numel(required)
        verifyTrue(testCase, isfolder(fullfile(testCase.TestData.Root, required{k})), ...
            sprintf('Missing required folder: %s', required{k}));
    end
end

function testPilotHasExactlyThreeTargets(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    verifyEqual(testCase, cfg.Targets.Names, ["KICK", "SNARE", "CRASH"]);
    verifyEqual(testCase, numel(unique(cfg.Targets.Names)), 3);
end

function testConfiguredAudioExists(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    for k = 1:numel(cfg.Targets.AudioFiles)
        verifyTrue(testCase, isfile(fullfile(cfg.Paths.Audio, ...
            cfg.Targets.AudioFiles(k))), ...
            sprintf('Missing audio sample: %s', cfg.Targets.AudioFiles(k)));
    end
end

function testWorkingKickBaselineIsPreserved(testCase)
    kickFile = fullfile(testCase.TestData.Root, 'drumRunTests', ...
        'kickOnly', 'mainKickIMUFast.m');
    verifyTrue(testCase, isfile(kickFile));
    source = fileread(kickFile);
    verifyNotEmpty(testCase, regexp(source, 'WAIT_RETURN_TRAVEL', 'once'));
    verifyNotEmpty(testCase, regexp(source, 'MinReturnTravelRad', 'once'));
    verifyNotEmpty(testCase, regexp(source, 'MinInterHitSeconds', 'once'));
end

function testAccelerometerCalibrationMath(testCase)
    raw = [1.10, -0.10, 0.05; -0.90, 0.90, 0.05];
    calibration.accelOffsetG = [0.10, -0.10, 0.05];
    calibration.accelScale = [1.00, 1.00, 1.00];
    corrected = applyMPU6050AccelCalibration(raw, calibration);
    verifyEqual(testCase, corrected, [1, 0, 0; -1, 1, 0], ...
        'AbsTol', 1e-12);
end

function testDualKickPipelineExists(testCase)
    root = testCase.TestData.Root;
    requiredFiles = { ...
        fullfile('drumRunTests', 'kickOnly', 'mainDualKickIMU.m'), ...
        fullfile('calibration', 'calibrateDualIMUSession.m'), ...
        fullfile('IMU_subsystem', 'setupMPU6050Pair.m'), ...
        fullfile('IMU_subsystem', 'readMPU6050Sample.m'), ...
        fullfile('classifiers', 'ruleBased', 'initializeKickDetector.m'), ...
        fullfile('classifiers', 'ruleBased', 'updateKickDetector.m'), ...
        fullfile('audio', 'createKickPlayerBank.m')};
    for fileIndex = 1:numel(requiredFiles)
        verifyTrue(testCase, isfile(fullfile(root, requiredFiles{fileIndex})), ...
            sprintf('Missing dual-kick component: %s', requiredFiles{fileIndex}));
    end
end

function testBothIMUsAreConfigured(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    verifyEqual(testCase, cfg.IMU.Addresses, ["0x68", "0x69"]);
    verifyEqual(testCase, numel(cfg.IMU.HandNames), 2);
end

function testSessionLoggerWritesSchema(testCase)
    temporaryFolder = string(tempname);
    mkdir(temporaryFolder);
    cleanup = onCleanup(@()rmdir(temporaryFolder, 's'));
    rawFile = fullfile(temporaryFolder, 'raw.mat');
    metadataFile = fullfile(temporaryFolder, 'metadata.mat');
    metadata.sessionId = "test_session";
    metadata.completedNormally = false;
    logger = DualIMUSessionLogger(rawFile, metadataFile, metadata, 2);
    for sampleIndex = 1:3
        pairVector = sampleIndex * ones(1, 3, 2);
        logger.append(sampleIndex / 10, pairVector, pairVector, ...
            pairVector, sampleIndex * ones(1, 2), pairVector, ...
            logical([sampleIndex == 2, sampleIndex == 3]));
    end
    logger.CompletedNormally = true;
    logger.close();

    recorded = load(rawFile);
    verifyEqual(testCase, size(recorded.accelRaw_g), [3, 3, 2]);
    verifyEqual(testCase, recorded.kickHit, ...
        logical([0 0; 1 0; 0 1]));
    verifyEqual(testCase, recorded.frameValid, true(3, 2));
    verifyTrue(testCase, all(isnan(recorded.loopDuration_s)));
    verifyEqual(testCase, recorded.targetIndex, zeros(3, 2, 'uint8'));
    session = load(metadataFile, 'metadata');
    verifyTrue(testCase, session.metadata.completedNormally);
    verifyEqual(testCase, session.metadata.loggedSamples, 3);
    verifyEqual(testCase, session.metadata.kickCount, [1, 1]);
end

function testSessionLoggerWritesSynchronizedEMG(testCase)
    temporaryFolder = string(tempname);
    mkdir(temporaryFolder);
    cleanup = onCleanup(@()rmdir(temporaryFolder, 's'));
    rawFile = fullfile(temporaryFolder, 'raw.mat');
    metadataFile = fullfile(temporaryFolder, 'metadata.mat');
    metadata.sessionId = "combined_test";
    logger = DualIMUSessionLogger(rawFile, metadataFile, metadata, 2, 4);
    pairVector = ones(1, 3, 2);
    logger.append(0.1, pairVector, pairVector, pairVector, ones(1, 2), ...
        pairVector, false(1, 2), true(1, 2), 0.1, uint8([2, 0]), ...
        [0.01, 0.03], [1, 2, 3, 4], [0.04, 0.08]);
    logger.close();

    recorded = load(rawFile);
    verifyEqual(testCase, recorded.emgVoltage_V, [1, 2, 3, 4]);
    verifyEqual(testCase, recorded.imuTimestamp_s, 0.02, 'AbsTol', eps);
    verifyEqual(testCase, recorded.emgTimestamp_s, 0.06, 'AbsTol', eps);
    verifyEqual(testCase, recorded.imuReadStart_s, 0.01);
    verifyEqual(testCase, recorded.emgReadEnd_s, 0.08);
end
