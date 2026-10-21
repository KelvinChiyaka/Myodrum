function tests = testMLDataCollection
% TESTMLDATACOLLECTION Verify the hardware-independent collection plan.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'classifiers', 'machineLearning'));
    testCase.TestData.Root = root;
end

function testDefaultScheduleIsBalanced(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    schedule = buildMLCollectionSchedule(cfg.IMU.HandNames, ...
        cfg.Targets.Names, cfg.MachineLearning.CollectionRounds, ...
        cfg.MachineLearning.HitsPerClassPerHandPerRound, ...
        cfg.MachineLearning.NoHitMinimumSeconds);

    verifyEqual(testCase, height(schedule), 24);
    verifyEqual(testCase, sum(double(schedule.ExpectedHitCount)), 360);
    verifyEqual(testCase, unique(schedule.Label)', ...
        sort(cfg.MachineLearning.ClassNames));
    for currentRound = 1:cfg.MachineLearning.CollectionRounds
        for hand = 1:numel(cfg.IMU.HandNames)
            rows = schedule.Round == currentRound & ...
                schedule.HandIndex == hand;
            verifyEqual(testCase, sort(schedule.Label(rows))', ...
                sort(cfg.MachineLearning.ClassNames));
        end
    end
end

function testLeftOnlyScheduleHasNoRightBlocks(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root, "left-three-emg");
    schedule = buildMLCollectionSchedule( ...
        cfg.IMU.HandNames(cfg.IMU.ActiveHandIndices), ...
        cfg.Targets.Names, cfg.MachineLearning.CollectionRounds, ...
        cfg.MachineLearning.HitsPerClassPerHandPerRound, ...
        cfg.MachineLearning.NoHitMinimumSeconds);
    verifyEqual(testCase, height(schedule), 12);
    verifyEqual(testCase, unique(schedule.Hand), "LEFT");
    verifyEqual(testCase, sum(double(schedule.ExpectedHitCount)), 180);
end

function testNoHitBlocksHaveDurationAndNoExpectedHits(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    schedule = buildMLCollectionSchedule(cfg.IMU.HandNames, ...
        cfg.Targets.Names, 2, 7, 12);
    negative = schedule.Label == "NO_HIT";
    verifyEqual(testCase, schedule.ExpectedHitCount(negative), ...
        zeros(sum(negative), 1, 'uint16'));
    verifyEqual(testCase, schedule.MinimumDuration_s(negative), ...
        12 * ones(sum(negative), 1));
    verifyEqual(testCase, schedule.ExpectedHitCount(~negative), ...
        7 * ones(sum(~negative), 1, 'uint16'));
end

function testCollectorDoesNotContainAudioPlayback(testCase)
    collector = fullfile(testCase.TestData.Root, 'drumRunTests', ...
        'machineLearning', 'mainMLDataCollection.m');
    verifyTrue(testCase, isfile(collector));
    source = fileread(collector);
    verifyEmpty(testCase, regexp(source, 'audioplayer|playLabel|play\(', 'once'));
    verifyNotEmpty(testCase, regexp(source, 'mlTrainingStream\.mat', 'once'));
    verifyNotEmpty(testCase, regexp(source, 'blockManifest', 'once'));
end

function testCollectorDrainsStreamDuringCalibrationPrompts(testCase)
    collector = fullfile(testCase.TestData.Root, 'drumRunTests', ...
        'machineLearning', 'mainMLDataCollection.m');
    source = fileread(collector);
    verifyNotEmpty(testCase, regexp(source, ...
        'setReferencePrompt\(fig, cfg\.IMU\.HandNames\(hand\)', 'once'));
    verifyNotEmpty(testCase, regexp(source, ...
        'action = waitForStart\(reader, fig, cfg\)', 'once'));
    verifyNotEmpty(testCase, regexp(source, ...
        'discardPendingFrames\(reader, cfg\)', 'once'));
    verifyEmpty(testCase, regexp(source, ...
        'EMG REFERENCE[\s\S]{0,400}input\(', 'once'));
    verifyNotEmpty(testCase, regexp(source, ...
        'MLCollectionStreamRestart', 'once'));
end

function testSynchronizedBlockPackaging(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    frames.sequence = uint16([1; 2]);
    frames.timestamp_us = uint32([1000; 6000]);
    frames.time_s = [0; 0.005];
    frames.status = uint8([0; 0]);
    frames.frameValid = true(2, 2);
    frames.accelG = zeros(2, 3, 2);
    frames.accelG(:, 3, :) = 1;
    frames.gyroDps = repmat(reshape([2 3 4; 5 6 7], 2, 3, 1), 1, 1, 2);
    frames.temperatureC = 25 * ones(2, 2);
    frames.emgAdc = uint16([0 100 200 300; 400 500 600 700]);
    calibration = struct('accelOffsetG', [0 0 0], ...
        'accelScale', [1 1 1]);
    profile.baseline_V = zeros(1, 4);
    profile.activeSpan_V = ones(1, 4);
    schedule = buildMLCollectionSchedule(cfg.IMU.HandNames, ...
        cfg.Targets.Names, 1, 10, 10);

    block = prepareMLCollectionBlock(frames, schedule(1,:), ...
        {calibration, calibration}, zeros(2, 3), profile, cfg);

    verifyEqual(testCase, block.sequenceNumber, frames.sequence);
    verifyEqual(testCase, block.accelCalibrated_g, frames.accelG);
    verifyEqual(testCase, block.gyroCorrected_rad_s, ...
        deg2rad(frames.gyroDps), 'AbsTol', 1e-12);
    verifyEqual(testCase, block.emgVoltage_V, ...
        double(frames.emgAdc) * 5 / 1023, 'AbsTol', 1e-12);
    verifyEqual(testCase, block.blockId, ...
        repmat(schedule.BlockId(1), 2, 1));
    verifyEqual(testCase, block.blockClassIndex, ...
        repmat(schedule.ClassIndex(1), 2, 1));
end
