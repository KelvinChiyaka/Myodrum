function tests = testMLFeatureExtraction
% TESTMLFEATUREEXTRACTION Verify grouped, balanced window preparation.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'common'));
    addpath(fullfile(root,'classifiers','machineLearning'));
    testCase.TestData.Root = root;
end

function testSyntheticBlocksProduceBalancedFiniteFeatures(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    cfg.MachineLearning.NoHitWindowsPerBlock = 3;
    sampleRate = cfg.EMG.CombinedStreamRateHz;
    blockLength = 600;
    count = 2 * blockLength;
    recorded.time_s = (0:count-1)' / sampleRate;
    recorded.frameValid = true(count,2);
    recorded.accelCalibrated_g = zeros(count,3,2);
    recorded.accelCalibrated_g(:,3,:) = 1;
    recorded.gyroCorrected_rad_s = zeros(count,3,2);
    recorded.emgNormalized = zeros(count,4);
    recorded.blockId = uint16([ones(blockLength,1);2*ones(blockLength,1)]);
    recorded.round = ones(count,1,'uint8');
    recorded.handIndex = ones(count,1,'uint8');
    recorded.blockClassIndex = uint8([ones(blockLength,1); ...
        2*ones(blockLength,1)]);
    recorded.classNames = ["NO_HIT","KICK","SNARE","CRASH"];
    peaks = blockLength + [120,300,480];
    for peak = peaks
        rows = peak-2:peak+2;
        recorded.accelCalibrated_g(rows,1,1) = [0.5;1;2;1;0.5];
        recorded.gyroCorrected_rad_s(rows,2,1) = [1;2;4;2;1];
        recorded.emgNormalized(rows,1:2) = 0.8;
    end
    recorded.blockManifest = table(uint16([1;2]),uint8([1;1]), ...
        uint8([1;1]),["LEFT";"LEFT"],uint8([1;2]), ...
        ["NO_HIT";"KICK"],uint16([0;3]),uint32([blockLength;blockLength]), ...
        true(2,1),'VariableNames',{'BlockId','Round','HandIndex','Hand', ...
        'ClassIndex','Label','ExpectedHitCount','RecordedFrames','Completed'});

    [features,diagnostics,names] = extractMLSessionFeatures( ...
        recorded,"synthetic_session",cfg);
    verifyEqual(testCase,height(features),6);
    verifyEqual(testCase,sum(features.Label=="NO_HIT"),3);
    verifyEqual(testCase,sum(features.Label=="KICK"),3);
    verifyTrue(testCase,all(isfinite(features{:,cellstr(names)}),'all'));
    verifyEqual(testCase,height(diagnostics),2);
    verifyEqual(testCase,features.SessionRoundGroup, ...
        repmat("synthetic_session_R1",6,1));
    verifyFalse(testCase,any(contains(names, ...
        ["Session","Label","Block","HandIndex"])));
    detected = features.WindowCenter_s(features.Label=="KICK");
    verifyEqual(testCase,detected,recorded.time_s(peaks),'AbsTol',0.03);
end

function testLeftThreeChannelFeatureDefinition(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root, "left-three-emg");
    count = 61;
    time = (0:count-1)' / cfg.EMG.CombinedStreamRateHz;
    accel = [zeros(count,2), ones(count,1)];
    gyro = zeros(count,3);
    emg = zeros(count,4);
    emg(:,1:3) = 0.5;
    [values,names] = computeMLWindowFeatures( ...
        accel,gyro,emg,time,1,cfg);
    verifyEqual(testCase,numel(values),62);
    verifyTrue(testCase,any(names=="SameEMG3Max"));
    verifyFalse(testCase,any(startsWith(names,"OtherEMG")));
end
