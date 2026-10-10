function cfg = myoDrumConfig(projectRoot)
% MYODRUMCONFIG Return the shared MyoDrum hardware and experiment settings.
%
% Keep settings used by more than one subsystem here. Hardware-specific
% tuning that belongs to a single detector may remain beside that detector
% until it is experimentally validated and promoted into the shared pipeline.

    if nargin < 1 || strlength(string(projectRoot)) == 0
        projectRoot = fileparts(fileparts(mfilename('fullpath')));
    end
    projectRoot = char(projectRoot);

    cfg.Project.Name = "MyoDrum";
    cfg.Project.DisplayName = "MyoDrum: EMG and IMU Virtual Drum Instrument";
    cfg.Project.SchemaVersion = "0.1.0";

    cfg.Hardware.COMPort = "COM7";
    cfg.Hardware.Board = "Mega2560";

    cfg.IMU.Addresses = ["0x68", "0x69"];
    cfg.IMU.HandNames = ["LEFT", "RIGHT"];
    cfg.IMU.SensorOutputRateHz = 50;
    cfg.IMU.AccelerometerRangeG = 2;
    cfg.IMU.GyroscopeRangeDps = 250;

    % The first controlled pilot deliberately uses three separable targets.
    cfg.Targets.Names = ["KICK", "SNARE", "CRASH"];
    cfg.Targets.CalibrationHitsPerTargetPerHand = 10;
    cfg.Targets.AudioFiles = ["kick.wav", "snare1.wav", "ridecrash.wav"];

    % EMG remains optional until its acquisition and channel map are verified.
    cfg.EMG.Enabled = false;
    cfg.EMG.AnalogPins = strings(0, 1);
    cfg.EMG.HitThresholdVolts = 3.0;

    cfg.Paths.ProjectRoot = string(projectRoot);
    cfg.Paths.Audio = string(fullfile(projectRoot, 'audio'));
    cfg.Paths.DataRoot = string(fullfile(projectRoot, 'data'));
    cfg.Paths.RawData = string(fullfile(projectRoot, 'data', 'raw'));
    cfg.Paths.ProcessedData = string(fullfile(projectRoot, 'data', 'processed'));
    cfg.Paths.Metadata = string(fullfile(projectRoot, 'data', 'metadata'));
    cfg.Paths.Models = string(fullfile(projectRoot, 'data', 'models'));
end
