function tests = testProjectStructure
% TESTPROJECTSTRUCTURE Hardware-independent checks for the repository layout.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    testCase.TestData.Root = root;
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
