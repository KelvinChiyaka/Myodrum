function tests = testEMGSubsystem
% TESTEMGSUBSYSTEM Hardware-independent EMG recorder tests.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'drumRunTests', 'emgOnly'));
    addpath(fullfile(root, 'tests', 'helpers'));
    testCase.TestData.Root = root;
end

function testConfigPinMapAndShapes(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    expectedPins = ["A0"; "A2"; "A3"; "A4"];
    verifyEqual(testCase, cfg.EMG.AnalogPins, expectedPins);
    verifyEqual(testCase, numel(unique(cfg.EMG.AnalogPins)), ...
        numel(cfg.EMG.AnalogPins));
    verifyEqual(testCase, size(cfg.EMG.ChannelNames), size(expectedPins));
    verifyEqual(testCase, size(cfg.EMG.Hands), size(expectedPins));
    verifyFalse(testCase, cfg.EMG.Enabled);
    verifyFalse(testCase, any(ismember(["A1"; "A5"], cfg.EMG.AnalogPins)));
end

function testLeftThreeEMGProfileUsesOnlyLeftChannels(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root, "left-three-emg");
    verifyEqual(testCase, cfg.IMU.ActiveHandIndices, 1);
    verifyEqual(testCase, cfg.EMG.AnalogPins, ...
        ["A0"; "A1"; "A2"; "A4"]);
    verifyEqual(testCase, cfg.EMG.ActiveChannelMask, ...
        [true; true; true; false]);
    verifyEqual(testCase, cfg.EMG.Hands, ...
        ["LEFT"; "LEFT"; "LEFT"; "UNUSED"]);
    verifyEqual(testCase, cfg.Fusion.RequiredActiveChannels, 2);
end

function testLeftThreeEMGFirmwareUsesConfiguredPins(testCase)
    firmware = fileread(fullfile(testCase.TestData.Root, 'firmware', ...
        'MyoDrumLeft3EMGStream', 'MyoDrumLeft3EMGStream.ino'));
    verifyNotEmpty(testCase, regexp(firmware, ...
        'EMG_PINS\[\]\s*=\s*\{A0,\s*A1,\s*A2,\s*A4\}', 'once'));
end

function testCoupledChannelDetection(testCase)
    waveform = sin((1:200)' / 9);
    values = [waveform, mod((1:200)', 17) / 17, ...
        cos((1:200)' / 7), waveform];
    result = findCoupledEMGChannels(values, ...
        ["LEFT_FOREARM"; "LEFT_TRICEPS"; ...
         "RIGHT_FOREARM"; "RIGHT_TRICEPS"]);
    verifyEqual(testCase, result.pairs, [1, 4]);
    verifyGreaterThan(testCase, result.correlations, 0.99);
end

function testLoggerSchemaAcrossBufferBoundary(testCase)
    folder = string(tempname);
    mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder, 's'));
    rawFile = fullfile(folder, 'emg.mat');
    metadataFile = fullfile(folder, 'metadata.mat');
    metadata.sessionId = "session_test";
    logger = EMGSessionLogger(rawFile, metadataFile, metadata, 4, 2);
    for row = 1:3
        logger.append(0.1 * row, row * ones(1, 4), 0.01, ...
            uint8(1 + (row > 1)), uint8(row - 1));
    end
    logger.CompletedNormally = true;
    logger.close();

    raw = load(rawFile);
    verifySize(testCase, raw.emgVoltage_V, [3, 4]);
    verifySize(testCase, raw.time_s, [3, 1]);
    verifyClass(testCase, raw.phase, 'uint8');
    verifyClass(testCase, raw.marker, 'uint8');
    saved = load(metadataFile, 'metadata');
    verifyEqual(testCase, saved.metadata.loggedSamples, 3);
    verifyTrue(testCase, saved.metadata.completedNormally);
    clear cleanup;
end

function testRestBaselineIsRobustToOutlier(testCase)
    values = [1, 2; 1, 2; 1, 2; 100, 2];
    baseline = computeRestBaseline(values);
    verifyEqual(testCase, baseline.median_V, [1, 2], 'AbsTol', 1e-12);
    verifyEqual(testCase, baseline.robustSigma_V, [0, 0], ...
        'AbsTol', 1e-12);
end

function testQualityFlags(testCase)
    cfg.FlatRangeVolts = 0.005;
    cfg.RailVolts = 4.8;
    cfg.RestSigmaWarnVolts = 0.15;
    values = [0, 0, 0, 0; 0, 2, 4.9, 0.5; 0, 1, 4.7, 0];
    rest.median_V = zeros(1, 4);
    rest.robustSigma_V = [0, 0, 0, 0.2];
    quality = summarizeEMGQuality(values, rest, cfg);
    verifyTrue(testCase, quality.neverResponded(1));
    verifyFalse(testCase, quality.neverResponded(2));
    verifyFalse(testCase, quality.railed(2));
    verifyFalse(testCase, quality.noisyRest(2));
    verifyTrue(testCase, quality.railed(3));
    verifyTrue(testCase, quality.noisyRest(4));
end

function testRecorderWritesOnlyTemporarySession(testCase)
    folder = string(tempname);
    mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder, 's'));
    cfg = temporaryConfig(testCase.TestData.Root, folder);
    fake = FakeEMGReader;
    notes = ["minimum"; "middle"; "maximum"; "two turns"];
    markerProvider = @(index, ~)scriptedMarker(index);
    mainEMGRecord(Config=cfg, Reader=@(pin)fake.read(pin), ...
        DurationSeconds=0.20, SkipRest=true, ShowPlot=false, ...
        GainNotes=notes, Notes="unit test", MarkerProvider=markerProvider);

    rawFiles = dir(fullfile(cfg.Paths.RawData, 'session_*', ...
        'emgAcquisition.mat'));
    metadataFiles = dir(fullfile(cfg.Paths.Metadata, 'sessions', ...
        'session_*', 'sessionMetadata.mat'));
    verifyNumElements(testCase, rawFiles, 1);
    verifyNumElements(testCase, metadataFiles, 1);
    raw = load(fullfile(rawFiles(1).folder, rawFiles(1).name));
    verifyEqual(testCase, size(raw.emgVoltage_V, 2), 4);
    verifyGreaterThan(testCase, size(raw.emgVoltage_V, 1), 2);
    verifyGreaterThan(testCase, diff(raw.time_s), zeros(numel(raw.time_s)-1, 1));
    verifyEqual(testCase, raw.marker(1:3), uint8([1; 2; 3]));
    saved = load(fullfile(metadataFiles(1).folder, metadataFiles(1).name), ...
        'metadata');
    verifyEqual(testCase, saved.metadata.channelGainNotes, notes);
    verifyEqual(testCase, saved.metadata.operatorNotes, "unit test");
    verifyEqual(testCase, saved.metadata.rateWarning, "");
    verifyTrue(testCase, saved.metadata.completedNormally);
    clear cleanup;
end

function marker = scriptedMarker(index)
    marker = uint8(0);
    if index <= 3
        marker = uint8(index);
    end
end

function testDefaultGainNotesAndSlowRateWarning(testCase)
    folder = string(tempname);
    mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder, 's'));
    cfg = temporaryConfig(testCase.TestData.Root, folder);
    cfg.EMG.MinRecommendedSetRateHz = 1000;
    fake = FakeEMGReader(0.002);
    mainEMGRecord(Config=cfg, Reader=@(pin)fake.read(pin), ...
        DurationSeconds=0.08, SkipRest=true, ShowPlot=false);
    metadataFiles = dir(fullfile(cfg.Paths.Metadata, 'sessions', ...
        'session_*', 'sessionMetadata.mat'));
    saved = load(fullfile(metadataFiles(1).folder, metadataFiles(1).name), ...
        'metadata');
    verifyEqual(testCase, saved.metadata.channelGainNotes, ...
        repmat("unrecorded", 4, 1));
    verifyNotEmpty(testCase, saved.metadata.rateWarning);
    clear cleanup;
end

function cfg = temporaryConfig(root, folder)
    cfg = myoDrumConfig(root);
    cfg.Paths.DataRoot = folder;
    cfg.Paths.RawData = fullfile(folder, 'raw');
    cfg.Paths.ProcessedData = fullfile(folder, 'processed');
    cfg.Paths.Metadata = fullfile(folder, 'metadata');
    cfg.Paths.Models = fullfile(folder, 'models');
    cfg.EMG.PreflightSeconds = 0.005;
    cfg.EMG.MinRecommendedSetRateHz = 1;
    cfg.Logging.BufferSamples = 3;
end
