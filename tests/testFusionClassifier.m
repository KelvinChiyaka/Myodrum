function tests = testFusionClassifier
% TESTFUSIONCLASSIFIER Pure rule-based pending-hit tests.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'common'));
    addpath(fullfile(root,'classifiers','ruleBased'));
    testCase.TestData.Cfg = myoDrumConfig(root);
    testCase.TestData.Hands = ["LEFT";"LEFT";"RIGHT";"RIGHT"];
end

function testImmediateConfirmation(testCase)
    state=initializeFusionClassifier;
    [~,decision,events]=step(state,0,true(1,2),["KICK","SNARE"], ...
        [0.4 0.1 0.1 0.5],testCase);
    verifyEqual(testCase,decision.accepted,true(1,2));
    verifyEqual(testCase,[events.accepted],true(1,2));
    verifyEqual(testCase,string({events.reason}),["ACCEPTED","ACCEPTED"]);
end

function testDelayedConfirmationAndLatency(testCase)
    state=initializeFusionClassifier;
    [state,decision,events]=step(state,1,[true false],["CRASH",""], ...
        zeros(1,4),testCase);
    verifyFalse(testCase,any(decision.accepted)); verifyEmpty(testCase,events);
    [~,decision,events]=step(state,1.055,false(1,2),strings(1,2), ...
        [0.3 0 0 0],testCase);
    verifyTrue(testCase,decision.accepted(1));
    verifyEqual(testCase,events.decisionLatency_s,0.055,'AbsTol',1e-12);
end

function testTimeoutAndUncertainOrientation(testCase)
    state=initializeFusionClassifier;
    [state,~,~]=step(state,0,[true false],["KICK",""],zeros(1,4),testCase);
    [state,decision,events]=step(state,0.121,false(1,2),strings(1,2), ...
        zeros(1,4),testCase);
    verifyTrue(testCase,decision.rejected(1));
    verifyEqual(testCase,events.reason,"EMG_TIMEOUT");
    [~,decision,events]=step(state,1,[false true],["",""], ...
        [0 0 0.8 0.8],testCase);
    verifyTrue(testCase,decision.rejected(2));
    verifyEqual(testCase,events.reason,"UNCERTAIN_ORIENTATION");
end

function testDuplicateGuardIsPerHand(testCase)
    state=initializeFusionClassifier;
    [state,~,~]=step(state,0,true(1,2),["KICK","KICK"], ...
        [1 1 1 1],testCase);
    [~,decision,events]=step(state,0.1,[true false],["SNARE",""], ...
        [1 1 0 0],testCase);
    verifyTrue(testCase,decision.rejected(1));
    verifyEqual(testCase,events.reason,"DUPLICATE_GUARD");
end

function [state,decision,events]=step(state,time,candidates,labels,emg,testCase)
    cfg=testCase.TestData.Cfg;
    [state,decision,events]=updateFusionClassifier(state,time,candidates, ...
        labels,emg,testCase.TestData.Hands,true(4,1),cfg.Fusion);
end
