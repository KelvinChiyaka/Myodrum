function tests = testAzimuthTracker
% TESTAZIMUTHTRACKER Hardware-free tests for azimuth drum selection.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'calibration'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
end

function testRecoversCentresAndDrift(testCase)
    rng(11);
    names = ["SNARE", "KICK", "CRASH"];
    truth = [40, 0, -40];
    [az, t, hand, labels] = syntheticCalibration(truth, 0.35, 2.0, 12);
    model = buildAzimuthModel(az, t, hand, labels, names);
    for h = 1:2
        verifyTrue(testCase, model.hands(h).ok, model.hands(h).reason);
        verifyEqual(testCase, model.hands(h).driftDegPerSec, 0.35, ...
            'AbsTol', 0.15);
        % centres refer to the end of calibration: truth + drift*tEnd
        tEnd = max(t(hand == h));
        verifyEqual(testCase, model.hands(h).centersDeg, ...
            truth + 0.35 * tEnd, 'AbsTol', 6);
    end
end

function testTrackerFollowsDriftWhereStaticClassifierFails(testCase)
    rng(12);
    names = ["SNARE", "KICK", "CRASH"];
    truth = [40, 0, -40];
    [az, t, hand, labels] = syntheticCalibration(truth, 0.0, 2.0, 12);
    model = buildAzimuthModel(az, t, hand, labels, names);

    tracker = DrumAzimuthTracker(model);
    driftRate = 0.5;                         % deg/s of heading wander
    trials = 120;
    correctTracked = 0;
    correctStatic = 0;
    for k = 1:trials
        drift = driftRate * k * 1.0;          % 1 s between hits
        target = randi(3);
        measured = truth(target) + drift + randn * 3;
        label = tracker.classify(1, measured, true);
        correctTracked = correctTracked + double(label == names(target));
        staticLabel = tracker.Model.targetNames( ...
            argminDistance(model.hands(1), measured));
        correctStatic = correctStatic + double(staticLabel == names(target));
    end
    verifyGreaterThan(testCase, correctTracked / trials, 0.85);
    verifyLessThan(testCase, correctStatic / trials, 0.6);
end

function testWrapsAcrossPlusMinus180(testCase)
    rng(13);
    names = ["SNARE", "KICK", "CRASH"];
    truth = [-160, 170, 140];                 % straddles the +-180 seam
    [az, t, hand, labels] = syntheticCalibration(truth, 0.0, 2.0, 10);
    model = buildAzimuthModel(az, t, hand, labels, names);
    tracker = DrumAzimuthTracker(model);
    verifyTrue(testCase, model.hands(1).ok, model.hands(1).reason);
    verifyEqual(testCase, tracker.classify(1, -158, false), "SNARE");
    verifyEqual(testCase, tracker.classify(1, 172, false), "KICK");
    verifyEqual(testCase, tracker.classify(1, 138, false), "CRASH");
end

function testOverlappingDrumsAreRejected(testCase)
    rng(14);
    names = ["SNARE", "KICK", "CRASH"];
    truth = [6, 0, -6];                       % drums nearly on top of each other
    [az, t, hand, labels] = syntheticCalibration(truth, 0.0, 2.0, 12);
    model = buildAzimuthModel(az, t, hand, labels, names);
    verifyFalse(testCase, any([model.hands.ok]));
end

function testWildHitCannotRunOffsetAway(testCase)
    rng(15);
    names = ["SNARE", "KICK", "CRASH"];
    truth = [40, 0, -40];
    [az, t, hand, labels] = syntheticCalibration(truth, 0.0, 2.0, 12);
    model = buildAzimuthModel(az, t, hand, labels, names);
    tracker = DrumAzimuthTracker(model);
    for k = 1:40
        tracker.classify(1, 120 + randn * 30, true);   % nonsense azimuths
    end
    verifyLessThanOrEqual(testCase, abs(tracker.OffsetDeg(1)), 90);
    tracker.reset();
    verifyEqual(testCase, tracker.OffsetDeg, [0 0]);
end

% ---------------------------------------------------------------- helpers

function index = argminDistance(handModel, azimuth)
    d = abs(mod(azimuth - handModel.centersDeg + 180, 360) - 180) ./ ...
        handModel.spreadDeg;
    [~, index] = min(d);
end

function [az, t, hand, labels] = syntheticCalibration(truth, driftDegPerSec, ...
        secondsPerHit, rounds)
    names = ["SNARE", "KICK", "CRASH"];
    order = [1 2 3 2];
    az = zeros(0, 1); t = zeros(0, 1); hand = zeros(0, 1); labels = strings(0, 1);
    for h = 1:2
        clock = 0;
        for r = 1:rounds
            for target = order
                clock = clock + secondsPerHit;
                measured = truth(target) + driftDegPerSec * clock + randn * 3;
                az(end + 1, 1) = mod(measured + 180, 360) - 180; %#ok<AGROW>
                t(end + 1, 1) = clock; %#ok<AGROW>
                hand(end + 1, 1) = h; %#ok<AGROW>
                labels(end + 1, 1) = names(target); %#ok<AGROW>
            end
        end
    end
end
