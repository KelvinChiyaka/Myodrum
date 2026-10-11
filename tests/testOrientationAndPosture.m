function tests = testOrientationAndPosture
% TESTORIENTATIONANDPOSTURE Hardware-free tests for the orientation estimator
% and the hand/posture drum selection.
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

function testInitialisesFromGravityAndConverges(testCase)
    up = [0.5, 0, sqrt(1 - 0.25)];      % tilted 30 deg about Y
    est = GravityAzimuthEstimator();
    for k = 0:20
        est.update(up, [0 0 0], 0.07 * k);
    end
    verifyEqual(testCase, est.GravityBody, up, 'AbsTol', 1e-6);
    verifyEqual(testCase, est.PitchDeg(), -30, 'AbsTol', 1e-6);
    verifyTrue(testCase, est.Converged);
end

function testNotConvergedImmediately(testCase)
    est = GravityAzimuthEstimator();
    est.update([0 0 1], [0 0 0], 0);
    est.update([0 0 1], [0 0 0], 0.07);
    verifyFalse(testCase, est.Converged);
end

function testRotationAboutVerticalGivesAzimuth(testCase)
    est = GravityAzimuthEstimator();
    rate = deg2rad(40);
    t = 0;
    est.update([0 0 1], [0 0 rate], t);
    for k = 1:43                          % about 3 s at 14 Hz
        t = t + 0.07;
        est.update([0 0 1], [0 0 rate], t);
    end
    verifyEqual(testCase, est.AzimuthDeg(), 40 * t, 'AbsTol', 1);
    verifyEqual(testCase, est.PitchDeg(), 0, 'AbsTol', 0.5);
end

function testTiltTracksRotationAboutHorizontalAxis(testCase)
    est = GravityAzimuthEstimator();
    rate = 1.0;                           % rad/s about sensor X
    t = 0;
    est.update([0 0 1], [rate 0 0], t);
    for k = 1:14
        t = t + 0.07;
        theta = rate * t;
        est.update([0, sin(theta), cos(theta)], [rate 0 0], t);
    end
    verifyEqual(testCase, est.RollDeg(), rad2deg(rate * t), 'AbsTol', 1.5);
end

function testImpactAccelerationDoesNotMoveTilt(testCase)
    est = GravityAzimuthEstimator();
    t = 0;
    for k = 1:20
        est.update([0 0 1], [0 0 0], t);
        t = t + 0.07;
    end
    before = est.GravityBody;
    for k = 1:4                           % violent stroke and impact
        est.update([0.9, 0.4, 2.2], [0 3.5 0], t);
        t = t + 0.07;
    end
    est.update([0.9, 0.4, 2.2], [0 0 0], t);
    after = est.GravityBody;
    angleMovedDeg = acosd(max(-1, min(1, dot(before, after))));
    % The gyro swing itself (3.5 rad/s for 0.28 s) rotates gravity; the
    % bad accelerometer must not add a pull toward [0.9 0.4 2.2].
    expected = GravityAzimuthEstimator.rotate(before, [0 -3.5 * 0.07 * 3.5 0]);
    verifyLessThan(testCase, angleMovedDeg, 80);
    verifyGreaterThan(testCase, dot(after, expected), 0.0);
end

function testStrokeAxisRemovedFromAzimuth(testCase)
    % Stick flexes about sensor Y while Y is tilted 12 deg from horizontal.
    % The stroke is fast on the way down and slow on the way up, point
    % sampled at 14 Hz, which integrates to a false heading change unless
    % the stroke axis is left out of the azimuth integral.
    tiltDeg = 12;
    withAxis = simulateStrokes(2, tiltDeg);
    withoutAxis = simulateStrokes(0, tiltDeg);
    verifyLessThan(testCase, abs(withAxis), 2);
    verifyGreaterThan(testCase, abs(withoutAxis), abs(withAxis) + 3);
end

function testStillnessLearnsBiasAndFreezesAzimuth(testCase)
    est = GravityAzimuthEstimator();
    biasDps = 0.5;
    t = 0;
    for k = 1:200
        est.update([0 0 1], [0 0 deg2rad(biasDps)], t);
        t = t + 0.07;
    end
    verifyTrue(testCase, est.Still);
    verifyLessThan(testCase, abs(est.AzimuthDeg()), 1.5);
    verifyEqual(testCase, rad2deg(est.BiasRadPerSec(3)), biasDps, ...
        'AbsTol', 0.05);
end

function testRecenterAndSetAzimuth(testCase)
    est = GravityAzimuthEstimator();
    t = 0;
    est.update([0 0 1], [0 0 1], t);
    for k = 1:10
        t = t + 0.07;
        est.update([0 0 1], [0 0 1], t);
    end
    verifyGreaterThan(testCase, abs(est.AzimuthDeg()), 20);
    est.recenter();
    verifyEqual(testCase, est.AzimuthDeg(), 0, 'AbsTol', 1e-9);
    est.setAzimuthRelativeDeg(25);
    verifyEqual(testCase, est.AzimuthDeg(), 25, 'AbsTol', 1e-9);
end

function testPostureModelSeparatesCrash(testCase)
    rng(3);
    [vectors, hands, labels] = syntheticPosture(40, 35);
    model = buildPostureModel(vectors, hands, labels, ...
        ["SNARE", "KICK"], "CRASH");
    verifyTrue(testCase, all([model.hands.crashEnabled]));
    verifyGreaterThan(testCase, model.hands(1).trainingAccuracy, 0.85);

    normalLeft = unitVector(0, 0);
    crashLeft = unitVector(35, 0);
    verifyEqual(testCase, classifyPosture(model, 1, normalLeft), "SNARE");
    verifyEqual(testCase, classifyPosture(model, 2, normalLeft), "KICK");
    verifyEqual(testCase, classifyPosture(model, 1, crashLeft), "CRASH");
    verifyEqual(testCase, classifyPosture(model, 2, crashLeft), "CRASH");
end

function testOverlappingPostureDisablesCrashAndKeepsHandDrum(testCase)
    rng(4);
    [vectors, hands, labels] = syntheticPosture(40, 3);   % only 3 deg apart
    model = buildPostureModel(vectors, hands, labels, ...
        ["SNARE", "KICK"], "CRASH");
    verifyFalse(testCase, any([model.hands.crashEnabled]));
    verifyEqual(testCase, classifyPosture(model, 1, unitVector(3, 0)), ...
        "SNARE");
    verifyEqual(testCase, classifyPosture(model, 2, unitVector(3, 0)), ...
        "KICK");
end

function testHandAloneDecidesOrdinaryDrum(testCase)
    rng(5);
    [vectors, hands, labels] = syntheticPosture(40, 35);
    model = buildPostureModel(vectors, hands, labels, ...
        ["SNARE", "KICK"], "CRASH");
    % Whatever the normal posture does (including wandering around the
    % normal cluster), the hand still decides the sound: no yaw anywhere.
    for k = 1:50
        wander = unitVector(randn * 2, randn * 2);
        verifyEqual(testCase, classifyPosture(model, 1, wander), "SNARE");
        verifyEqual(testCase, classifyPosture(model, 2, wander), "KICK");
    end
end

function testGyroRangeScaleDecoding(testCase)
    raw = uint8([0 0, 0 0, 0 0, 0 0, 0 100, 0 0, 0 0]);
    at250 = decodeMPU6050Frame(raw, 131);
    at1000 = decodeMPU6050Frame(raw, 32.8);
    verifyEqual(testCase, at250.gyroDps(1), 100 / 131, 'AbsTol', 1e-12);
    verifyEqual(testCase, at1000.gyroDps(1), 100 / 32.8, 'AbsTol', 1e-12);
end

function testGravityVectorDrumsAvoidEulerSingularity(testCase)
    [gravity, gyro, hands, labels, names] = syntheticGravityDrums;
    model = buildGravityDrumModel(gravity, gyro, hands, labels, names);
    verifyTrue(testCase, all(model.okByHand));
    for hand = 1:2
        verifyEqual(testCase, classifyGravityDrum( ...
            model, hand, [1 0 0]), "KICK");
        verifyEqual(testCase, classifyGravityDrum( ...
            model, hand, [-1 0 0]), "SNARE");
        verifyEqual(testCase, classifyGravityDrum( ...
            model, hand, [0 0 1]), "CRASH");
    end
end

function testGyroDirectionRejectsButNeverRelabels(testCase)
    [gravity, gyro, hands, labels, names] = syntheticGravityDrums;
    model = buildGravityDrumModel(gravity, gyro, hands, labels, names);
    accepted = classifyGravityDrum(model, 1, [1 0 0], ...
        GyroPeakVector=[0 1 0], MaximumGyroAngleDeg=45);
    rejected = classifyGravityDrum(model, 1, [1 0 0], ...
        GyroPeakVector=[0 -1 0], MaximumGyroAngleDeg=45);
    verifyEqual(testCase, accepted, "KICK");
    verifyEqual(testCase, rejected, "");
end

% ---------------------------------------------------------------- helpers

function azimuthDeg = simulateStrokes(strokeAxis, tiltDeg)
    % Ground truth: the sensor flexes about its Y axis; gravity in the
    % sensor frame is rotated about Y by an oscillating angle on top of a
    % fixed tilt of Y away from the horizontal.
    est = GravityAzimuthEstimator();
    if strokeAxis > 0
        est.setStrokeAxis(strokeAxis);
    end
    fineDt = 0.001;
    sampleDt = 0.07;
    period = 0.7;
    t = 0;
    nextSample = 0;
    % body attitude: tilt about X so that body Y is tilted tiltDeg from
    % horizontal, then oscillate about body Y.
    baseUp = [0, sind(tiltDeg), cosd(tiltDeg)];
    for cycle = 1:6
        for step = 0:round(period / fineDt) - 1
            phase = step * fineDt / period;
            % fast down (first 25%), slow back up (remaining 75%)
            if phase < 0.25
                rateY = deg2rad(60) / (0.25 * period);
            else
                rateY = -deg2rad(60) / (0.75 * period);
            end
            if t >= nextSample
                gyro = [0, rateY, 0];
                up = currentUp(baseUp, cycle, step, fineDt, period);
                est.update(up, gyro, t);
                nextSample = nextSample + sampleDt;
            end
            t = t + fineDt;
        end
    end
    % flush to the end with the stick back at rest
    for k = 1:10
        est.update(baseUp, [0 0 0], t + k * sampleDt);
    end
    azimuthDeg = est.AzimuthDeg();
end

function up = currentUp(baseUp, ~, step, fineDt, period)
    phase = step * fineDt / period;
    if phase < 0.25
        angle = deg2rad(60) * (phase / 0.25);
    else
        angle = deg2rad(60) * (1 - (phase - 0.25) / 0.75);
    end
    % rotating the sensor about Y by +angle turns gravity by -angle about Y
    up = GravityAzimuthEstimator.rotate(baseUp, [0, -angle, 0]);
end

function [vectors, hands, labels] = syntheticPosture(countPerClass, crashOffsetDeg)
    % Normal playing posture near pitch 0; crash posture crashOffsetDeg
    % further in pitch. Both have about 2 deg of scatter.
    vectors = zeros(0, 3);
    hands = zeros(0, 1);
    labels = strings(0, 1);
    names = ["SNARE", "KICK"];
    for hand = 1:2
        for k = 1:countPerClass
            vectors(end + 1, :) = unitVector(randn * 2, randn * 2); %#ok<AGROW>
            hands(end + 1, 1) = hand; %#ok<AGROW>
            labels(end + 1, 1) = names(hand); %#ok<AGROW>
            vectors(end + 1, :) = unitVector( ...
                crashOffsetDeg + randn * 2, randn * 2); %#ok<AGROW>
            hands(end + 1, 1) = hand; %#ok<AGROW>
            labels(end + 1, 1) = "CRASH"; %#ok<AGROW>
        end
    end
end

function u = unitVector(pitchDeg, rollDeg)
    u = [-sind(pitchDeg), sind(rollDeg) * cosd(pitchDeg), ...
        cosd(rollDeg) * cosd(pitchDeg)];
    u = u / norm(u);
end


function [gravity, gyro, hands, labels, names] = syntheticGravityDrums
    names = ["KICK", "SNARE", "CRASH"];
    gravityCenters = [1 0 0; -1 0 0; 0 0 1];
    gyroCenters = [0 1 0; 0 0 1; 1 0 0];
    gravity = zeros(0, 3);
    gyro = zeros(0, 3);
    hands = zeros(0, 1);
    labels = strings(0, 1);
    for hand = 1:2
        for target = 1:3
            for repetition = 1:3
                perturbation = 0.01 * repetition * [1 -1 0.5];
                gravity(end + 1, :) = ...
                    gravityCenters(target, :) + perturbation; %#ok<AGROW>
                gyro(end + 1, :) = ...
                    gyroCenters(target, :) + perturbation; %#ok<AGROW>
                hands(end + 1, 1) = hand; %#ok<AGROW>
                labels(end + 1, 1) = names(target); %#ok<AGROW>
            end
        end
    end
end
