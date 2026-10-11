function mainMyoDrum(mode)
% MAINMYODRUM Top-level entry point for the MyoDrum MATLAB project.
%
%   mainMyoDrum                  Show the available project modes.
%   mainMyoDrum("kick-only")     Run the preserved working one-IMU prototype.
%   mainMyoDrum("dual-kick")     Calibrate/log both IMUs and play kick.
%   mainMyoDrum("three-drum")   Calibrate and play kick/snare/crash regions.
%   mainMyoDrum("imu-emg")      Record synchronized two-IMU + four-EMG data.
%   mainMyoDrum("imu-emg-stream") Run 200 Hz synchronized fusion firmware.
%   mainMyoDrum("emg")          Record the four EMG envelope channels.
%   mainMyoDrum("emg-stream")   Record EMG using high-rate Mega firmware.
%   mainMyoDrum("imu-diagnostics") Run the existing two-IMU diagnostic script.
%   mainMyoDrum("status")        Display the active hardware and target setup.
%
% The calibration, acquisition, classification, and audio algorithms live in
% their subsystem folders. This function intentionally remains a small session
% controller instead of duplicating those implementations.

    arguments
        mode (1,1) string = "help"
    end

    root = fileparts(mfilename('fullpath'));
    addpath(fullfile(root, 'common'));
    cfg = myoDrumConfig(root);

    switch lower(strtrim(mode))
        case {"help", "menu"}
            printHelp(cfg);

        case {"status", "project-status"}
            printStatus(cfg);

        case {"kick", "kick-only"}
            kickFolder = fullfile(root, 'drumRunTests', 'kickOnly');
            addpath(fullfile(root, 'IMU_subsystem'));
            addpath(kickFolder);
            fprintf(['Running the preserved kick-only baseline. Its tested ' ...
                'thresholds and duplicate-hit safeguard are unchanged.\n']);
            mainKickIMUFast;

        case {"dual-kick", "two-imu-kick"}
            addpath(fullfile(root, 'IMU_subsystem'));
            addpath(fullfile(root, 'calibration'));
            addpath(fullfile(root, 'classifiers', 'ruleBased'));
            addpath(fullfile(root, 'audio'));
            dualKickFolder = fullfile(root, 'drumRunTests', 'kickOnly');
            addpath(dualKickFolder);
            mainDualKickIMU;

        case {"three-drum", "three-drums"}
            addpath(fullfile(root, 'IMU_subsystem'));
            addpath(fullfile(root, 'calibration'));
            addpath(fullfile(root, 'classifiers', 'ruleBased'));
            addpath(fullfile(root, 'audio'));
            dualKickFolder = fullfile(root, 'drumRunTests', 'kickOnly');
            addpath(dualKickFolder);
            mainDualKickIMU("three-drum");

        case {"imu-emg", "three-drum-emg", "combined"}
            addpath(fullfile(root, 'IMU_subsystem'));
            addpath(fullfile(root, 'EMG_subsystem'));
            addpath(fullfile(root, 'calibration'));
            addpath(fullfile(root, 'classifiers', 'ruleBased'));
            addpath(fullfile(root, 'audio'));
            dualKickFolder = fullfile(root, 'drumRunTests', 'kickOnly');
            addpath(dualKickFolder);
            mainDualKickIMU("three-drum-emg");

        case {"imu-emg-stream", "combined-stream", "fusion"}
            addpath(fullfile(root, 'IMU_subsystem'));
            addpath(fullfile(root, 'EMG_subsystem'));
            addpath(fullfile(root, 'classifiers', 'ruleBased'));
            addpath(fullfile(root, 'audio'));
            combinedFolder = fullfile(root, 'drumRunTests', 'combined');
            addpath(combinedFolder);
            mainIMUEMGStream;

        case {"emg", "emg-only"}
            addpath(fullfile(root, 'EMG_subsystem'));
            emgFolder = fullfile(root, 'drumRunTests', 'emgOnly');
            addpath(emgFolder);
            mainEMGRecord;

        case {"emg-stream", "fast-emg"}
            addpath(fullfile(root, 'EMG_subsystem'));
            emgFolder = fullfile(root, 'drumRunTests', 'emgOnly');
            addpath(emgFolder);
            mainEMGStreamRecord;

        case {"imu", "imu-diagnostics"}
            imuFolder = fullfile(root, 'IMU_subsystem');
            addpath(imuFolder);
            addpath(fullfile(root, 'drumRunTests', 'diagnostics'));
            fprintf('Running the existing two-IMU diagnostic and calibration script.\n');
            mainIMUDiagnostics;

        otherwise
            error('MyoDrum:UnknownMode', ...
                'Unknown mode "%s". Run mainMyoDrum("help") for valid modes.', mode);
    end
end

function printHelp(cfg)
    fprintf('\n%s\n', cfg.Project.DisplayName);
    fprintf('Current stage: repository foundation and three-drum pilot.\n\n');
    fprintf('Available commands:\n');
    fprintf('  mainMyoDrum("status")          Show the active configuration.\n');
    fprintf('  mainMyoDrum("kick-only")       Run the working kick detector.\n');
    fprintf('  mainMyoDrum("dual-kick")       Calibrate/log both IMUs; play kick.\n');
    fprintf('  mainMyoDrum("three-drum")      Calibrate/play kick, snare, crash.\n');
    fprintf('  mainMyoDrum("imu-emg")         Record labelled IMU + EMG gestures.\n');
    fprintf('  mainMyoDrum("imu-emg-stream")  Run synchronized 200 Hz fusion.\n');
    fprintf('  mainMyoDrum("emg")             Record four EMG envelope channels.\n');
    fprintf('  mainMyoDrum("emg-stream")      Use high-rate Mega EMG firmware.\n');
    fprintf('  mainMyoDrum("imu-diagnostics") Run the existing IMU checks.\n\n');
    fprintf(['Three-drum mode calibrates independent pitch/roll regions for ' ...
        'both sticks; either hand can play every configured sound.\n']);
end

function printStatus(cfg)
    fprintf('\n%s configuration\n', cfg.Project.DisplayName);
    fprintf('Arduino: %s on %s\n', cfg.Hardware.Board, cfg.Hardware.COMPort);
    fprintf('IMU addresses: %s (%s), %s (%s)\n', ...
        cfg.IMU.Addresses(1), cfg.IMU.HandNames(1), ...
        cfg.IMU.Addresses(2), cfg.IMU.HandNames(2));
    fprintf('Pilot targets: %s\n', strjoin(cfg.Targets.Names, ', '));
    fprintf('EMG enabled: %s\n', string(cfg.EMG.Enabled));
    fprintf('EMG pins: %s\n', strjoin( ...
        cfg.EMG.ChannelNames + "=" + cfg.EMG.AnalogPins, ', '));
    fprintf('Session output root: %s\n\n', cfg.Paths.DataRoot);
end
