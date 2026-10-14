function tests = testLiveMLClassifier
% TESTLIVEMLCLASSIFIER Verify live prediction and mandatory EMG gating.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'common'));
    addpath(fullfile(root,'classifiers','machineLearning'));
    testCase.TestData.Root = root;
end

function testLiveCandidateRequiresMLAndBothEMGChannels(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    cfg.MachineLearning.LiveMinimumConfidence = 0;
    cfg.Fusion.MinimumInterAcceptedHitSeconds = 0;
    folder = string(tempname); mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    modelFile = createTestModel(folder, cfg);

    accepted = runCandidate(modelFile, cfg, 0.5, ...
        "machine-learning", "");
    verifyTrue(testCase, accepted.accepted(1));
    verifyEqual(testCase, accepted.labels(1), "KICK");
    verifyEqual(testCase, accepted.reason(1), "ACCEPTED");

    rejected = runCandidate(modelFile, cfg, 0, ...
        "machine-learning", "");
    verifyFalse(testCase, rejected.accepted(1));
    verifyTrue(testCase, rejected.rejected(1));
    verifyEqual(testCase, rejected.reason(1), "EMG_BELOW_THRESHOLD");
end

function testLiveDefaultsUseFiftyPercentConfidence(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    verifyEqual(testCase, ...
        cfg.MachineLearning.LiveMinimumConfidence, 0.50, ...
        'AbsTol', 1e-12);
end

function testOneDroppedFrameDoesNotInvalidateWindow(testCase)
    source = fileread(fullfile(testCase.TestData.Root, 'classifiers', ...
        'machineLearning', 'updateLiveMLClassifier.m'));
    verifyNotEmpty(testCase, regexp(source, ...
        'validRows = rows\(state\.frameValid\(rows,hand\)\)', 'once'));
    verifyNotEmpty(testCase, regexp(source, ...
        'numel\(validRows\) >= minimumRows', 'once'));
end

function testHybridRequiresRuleAndMLAgreement(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    cfg.MachineLearning.LiveMinimumConfidence = 0;
    cfg.Fusion.MinimumInterAcceptedHitSeconds = 0;
    folder = string(tempname); mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    modelFile = createTestModel(folder, cfg);

    agreement = runCandidate(modelFile, cfg, 0.5, "hybrid", "KICK");
    verifyTrue(testCase, agreement.accepted(1));
    verifyEqual(testCase, agreement.ruleLabels(1), "KICK");

    disagreement = runCandidate(modelFile, cfg, 0.5, ...
        "hybrid", "SNARE");
    verifyFalse(testCase, disagreement.accepted(1));
    verifyTrue(testCase, disagreement.rejected(1));
    verifyEqual(testCase, disagreement.reason(1), ...
        "HYBRID_DISAGREEMENT");
    verifyEqual(testCase, disagreement.labels(1), "KICK");
    verifyEqual(testCase, disagreement.ruleLabels(1), "SNARE");
end

function finalDecision = runCandidate( ...
        modelFile, cfg, emgLevel, classifierMode, ruleLabel)
    state = initializeLiveMLClassifier("unused", cfg, ...
        ModelFile=modelFile, ClassifierMode=classifierMode);
    finalDecision = struct;
    for index = 0:100
        time = index / cfg.EMG.CombinedStreamRateHz;
        accel = zeros(1,3,2); accel(1,1,:) = 1; accel(1,3,:) = 1;
        gyro = zeros(1,3,2);
        emg = emgLevel * ones(1,4);
        candidate = false(1,2);
        if index == 60, candidate(1) = true; end
        ruleLabels = strings(1,2);
        if candidate(1), ruleLabels(1) = ruleLabel; end
        [state, decision, events] = updateLiveMLClassifier(state, time, ...
            accel, gyro, emg, true(1,2), true(1,4), candidate, cfg, ...
            ruleLabels);
        if ~isempty(events), finalDecision = decision; end
    end
end

function modelFile = createTestModel(folder, cfg)
    time = (0:60)' / cfg.EMG.CombinedStreamRateHz;
    accel = [ones(numel(time),1), zeros(numel(time),1), ...
        ones(numel(time),1)];
    gyro = zeros(numel(time),3);
    emg = 0.5 * ones(numel(time),4);
    [values, names] = computeMLWindowFeatures(accel, gyro, emg, ...
        time, 1, cfg);
    predictors = repmat(values, 16, 1);
    labels = repmat(["NO_HIT";"KICK";"SNARE";"CRASH"],4,1);
    classOffset = repmat([0;1;2;3],4,1);
    predictors(:,1) = classOffset;
    training = array2table(predictors, 'VariableNames', cellstr(names));
    classifier = fitctree(training, categorical(labels), ...
        'MinLeafSize',1, 'MaxNumSplits',20);
    modelArtifact.classifier = compact(classifier);
    modelArtifact.featureVersion = cfg.MachineLearning.FeatureVersion;
    modelArtifact.predictorNames = names(:);
    modelArtifact.classNames = cfg.MachineLearning.ClassNames(:);
    modelArtifact.modelVersion = "test";
    modelFile = fullfile(folder, 'model.mat');
    save(modelFile, 'modelArtifact');
end
