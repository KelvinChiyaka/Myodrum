function tests = testIMUEMGLiveSessionBuffer
% TESTIMUEMGLIVESESSIONBUFFER Offline checks for constant-cost live logging.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    testCase.TestData.Root = root;
end

function testBufferGrowsWithoutLosingSamples(testCase)
    log = initializeIMUEMGLiveLog(2);
    decision.accepted = false(1,2);
    decision.rejected = false(1,2);
    decision.pendingActive = false(1,2);
    decision.emgScore = zeros(1,2);
    decision.reason = strings(1,2);

    for sample = 1:5
        frames = exampleFrames(sample);
        raw = sample * ones(1,3,2);
        decision.accepted = logical([mod(sample,2), 0]);
        if decision.accepted(1)
            decision.reason(1) = "ACCEPTED";
        else
            decision.reason(1) = "";
        end
        log = appendIMUEMGLiveLog(log, frames, 1, ...
            raw, raw + 1, raw + 2, raw + 3, ...
            sample * ones(1,4), 0.1 * sample * ones(1,4), ...
            logical([sample == 3, false]), decision, ...
            uint8([1, 0]), uint8([2, 0]), true(1,4), sample / 1000);
    end

    recorded = snapshotIMUEMGLiveLog(log);
    verifyEqual(testCase, log.sampleCount, 5);
    verifyGreaterThanOrEqual(testCase, log.capacity, 5);
    verifyEqual(testCase, recorded.time_s, (1:5)' / 200);
    verifyEqual(testCase, size(recorded.accelRaw_g), [5,3,2]);
    verifyEqual(testCase, recorded.hostProcessingLag_s, (1:5)' / 1000);
    verifyEqual(testCase, recorded.fusionDecisionCode(:,1), ...
        uint8([1;0;1;0;1]));
end

function testTailReturnsOnlyNewestRows(testCase)
    log = initializeIMUEMGLiveLog(4);
    decision.accepted = false(1,2);
    decision.rejected = false(1,2);
    decision.pendingActive = false(1,2);
    decision.emgScore = zeros(1,2);
    decision.reason = strings(1,2);
    for sample = 1:4
        frames = exampleFrames(sample);
        vectors = zeros(1,3,2);
        log = appendIMUEMGLiveLog(log, frames, 1, ...
            vectors, vectors, vectors, vectors, ...
            zeros(1,4), zeros(1,4), false(1,2), decision, ...
            zeros(1,2,'uint8'), zeros(1,2,'uint8'), true(1,4), 0);
    end
    recent = snapshotIMUEMGLiveLog(log, 3:4);
    verifyEqual(testCase, recent.time_s, (3:4)' / 200);
    verifyEqual(testCase, numel(recent.sequenceNumber), 2);
end

function frames = exampleFrames(sample)
    frames.time_s = sample / 200;
    frames.sequence = uint16(sample);
    frames.status = uint8(0);
    frames.frameValid = true(1,2);
    frames.temperatureC = [25, 25];
end
