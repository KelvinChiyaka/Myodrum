function mainMyoDrum(mode, classifier, liveMode)

    arguments
        mode (1,1) string = "menu"
        classifier (1,1) string = "prompt"
        liveMode (1,1) string = "prompt"
    end

    root = fileparts(mfilename('fullpath'));
    addpath(fullfile(root, 'common'));
    cfg = myoDrumConfig(root);
    mode = canonicalMode(mode);

    if mode == "menu"
        mode = promptForInterface(cfg);
        if mode == "exit"
            fprintf('MyoDrum closed.\n');
            return;
        end
    end

    if mode == "classifier"
        printClassifierStatus;
        return;
    end
    if mode == "emg-only"
        runEMGOnly(root);
        return;
    end
    if classifier == "prompt"
        classifier = promptForClassifier;
    end
    classifier = canonicalClassifier(classifier);
    requireImplementedCombination(mode, classifier);

    switch mode
        case "imu-only"
            runIMUOnly(root);
        case "imu-emg-4"
            runFourChannelFusion(root, liveMode, classifier);
        case "imu-emg-6"
            error('MyoDrum:NotImplemented', [ ...
                'Six-channel IMU + EMG is reserved for the next hardware ' ...
                'stage. Its pins, packet protocol and electrode map have ' ...
                'not yet been defined.']);
        otherwise
            error('MyoDrum:UnknownMode', 'Unknown interface "%s".', mode);
    end
end

function mode = promptForInterface(cfg)
    fprintf('\n%s\n', cfg.Project.DisplayName);
    fprintf('Select the sensor interface:\n');
    fprintf('  1. IMU only\n');
    fprintf('  2. EMG only (four channels)\n');
    fprintf('  3. IMU + EMG (four channels)\n');
    fprintf('  4. IMU + EMG (six channels) [future]\n');
    fprintf('  5. Classifier status\n');
    fprintf('  0. Exit\n');
    choice = input('Selection: ');
    modes = ["exit", "imu-only", "emg-only", "imu-emg-4", ...
        "imu-emg-6", "classifier"];
    if ~isscalar(choice) || ~isnumeric(choice) || ~isfinite(choice) || ...
            choice ~= fix(choice) || choice < 0 || choice > 5
        error('MyoDrum:InvalidMenuSelection', ...
            'Enter one integer from 0 to 5.');
    end
    mode = modes(choice + 1);
end

function classifier = promptForClassifier
    fprintf('\nSelect the classifier:\n');
    fprintf('  1. Rule based [available]\n');
    fprintf('  2. Machine learning [available]\n');
    fprintf('  3. Hybrid rule based + machine learning [available]\n');
    choice = input('Selection: ');
    classifiers = ["rule-based", "machine-learning", "hybrid"];
    if ~isscalar(choice) || ~isnumeric(choice) || ~isfinite(choice) || ...
            choice ~= fix(choice) || choice < 1 || choice > 3
        error('MyoDrum:InvalidClassifierSelection', ...
            'Enter one integer from 1 to 3.');
    end
    classifier = classifiers(choice);
end

function mode = canonicalMode(mode)
    mode = lower(strtrim(mode));
    switch mode
        case {"menu", "help"}
            mode = "menu";
        case {"imu", "imu-only"}
            mode = "imu-only";
        case {"emg", "emg-only"}
            mode = "emg-only";
        case {"imu-emg", "imu-emg-4", "imu-emg-stream"}
            mode = "imu-emg-4";
        case "imu-emg-6"
            mode = "imu-emg-6";
        case {"classifier", "classifiers"}
            mode = "classifier";
        otherwise
            error('MyoDrum:UnknownMode', ...
                ['Unknown interface "%s". Run mainMyoDrum without ' ...
                'arguments to open the startup menu.'], mode);
    end
end

function classifier = canonicalClassifier(classifier)
    classifier = lower(strtrim(classifier));
    switch classifier
        case {"rule", "rules", "rule-based"}
            classifier = "rule-based";
        case {"ml", "machine-learning"}
            classifier = "machine-learning";
        case {"both", "hybrid"}
            classifier = "hybrid";
        otherwise
            error('MyoDrum:UnknownClassifier', ...
                'Unknown classifier "%s".', classifier);
    end
end

function requireImplementedCombination(mode, classifier)
    if mode == "imu-emg-6"
        return;
    end
    if classifier ~= "rule-based"
        if ismember(classifier, ["machine-learning", "hybrid"]) && ...
                mode == "imu-emg-4"
            return;
        end
        error('MyoDrum:NotImplemented', [ ...
            'Classifier "%s" is a reserved future option. Select the ' ...
            'rule-based classifier for the current prototype.'], classifier);
    end
end

function runIMUOnly(root)
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'calibration'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    addpath(fullfile(root, 'audio'));
    addpath(fullfile(root, 'drumRunTests', 'kickOnly'));
    mainDualKickIMU("three-drum");
end

function runEMGOnly(root)
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'drumRunTests', 'emgOnly'));
    mainEMGStreamRecord;
end

function runFourChannelFusion(root, liveMode, classifier)
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    addpath(fullfile(root, 'calibration'));
    addpath(fullfile(root, 'audio'));
    addpath(fullfile(root, 'drumRunTests', 'combined'));
    addpath(fullfile(root, 'classifiers', 'machineLearning'));
    mainIMUEMGStream(liveMode, classifier);
end

function printClassifierStatus
    fprintf('\nMYODRUM CLASSIFIERS\n');
    fprintf('  1. Rule based: available and under validation.\n');
    fprintf(['  2. Machine learning: trained bagged trees available for ' ...
        'four-channel IMU + EMG live play.\n']);
    fprintf(['  3. Hybrid: rule and ML labels must agree; EMG remains ' ...
        'mandatory.\n\n']);
end
