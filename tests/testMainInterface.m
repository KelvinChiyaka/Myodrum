function tests = testMainInterface
% TESTMAININTERFACE Verify the public menu and low-rate live dashboard.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'common'));
    testCase.TestData.Root = root;
end

function testPublicInterfacesAndFuturePlaceholders(testCase)
    entry = fileread(fullfile(testCase.TestData.Root, 'mainMyoDrum.m'));
    verifyNotEmpty(testCase, regexp(entry, '"imu-only"', 'once'));
    verifyNotEmpty(testCase, regexp(entry, '"emg-only"', 'once'));
    verifyNotEmpty(testCase, regexp(entry, '"imu-emg-4"', 'once'));
    verifyNotEmpty(testCase, regexp(entry, '"imu-emg-6"', 'once'));
    verifyNotEmpty(testCase, regexp(entry, '"classifier"', 'once'));
    verifyEmpty(testCase, regexp(entry, '"ml-collect"', 'once'));
    verifyNotEmpty(testCase, regexp(entry, ...
        'Machine learning \[collect/train separately\]', 'once'));
    verifyTrue(testCase, isfile(fullfile(testCase.TestData.Root, ...
        'calibration', 'calibrateMachineLearningDataset.m')));
    verifyError(testCase, @()mainMyoDrum( ...
        "imu-emg-6", "rule-based"), 'MyoDrum:NotImplemented');
    verifyError(testCase, @()mainMyoDrum( ...
        "imu-only", "machine-learning"), 'MyoDrum:NotImplemented');
    verifyError(testCase, @()mainMyoDrum( ...
        "imu-emg-4", "machine-learning"), ...
        'MyoDrum:MachineLearningModelUnavailable');
end

function testLivePlayUsesConsoleFeedbackWithoutPlots(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    verifyFalse(testCase, cfg.Visualization.Enabled);
    verifyTrue(testCase, cfg.Runtime.PrintHitEvents);
end

function testDashboardAcceptsRollingSensorData(testCase)
    cfg = myoDrumConfig(testCase.TestData.Root);
    dashboard = createLiveSensorDashboard(cfg, Visible="off");
    cleanup = onCleanup(@()closeDashboard(dashboard));
    count = 1500;
    log.time_s = (0:count-1)' / cfg.EMG.CombinedStreamRateHz;
    log.accelCalibrated_g = zeros(count, 3, 2);
    log.accelCalibrated_g(:, 3, :) = 1;
    log.gyroCorrected_rad_s = zeros(count, 3, 2);
    log.gyroCorrected_rad_s(:, 1, 1) = 2;
    log.emgNormalized = zeros(count, 4);
    log.emgNormalized(:, 1) = linspace(0, 1, count);
    updateLiveSensorDashboard(dashboard, log, cfg, ...
        ["SNARE", "KICK"], [0.6, 0.8], "GO: LEFT SNARE");
    verifyLessThanOrEqual(testCase, ...
        numel(dashboard.AccelLines(1).XData), ...
        cfg.Visualization.MaximumPoints);
    verifyEqual(testCase, dashboard.AccelLines(1).YData, ...
        ones(size(dashboard.AccelLines(1).YData)), 'AbsTol', 1e-12);
    verifyTrue(testCase, contains(dashboard.AccelAxes.Title.String, 'SNARE'));
    verifyEqual(testCase, dashboard.PromptTitle.String, 'GO: LEFT SNARE');
end

function closeDashboard(dashboard)
    if isgraphics(dashboard.Figure)
        delete(dashboard.Figure);
    end
end
