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

    cfg.Calibration.ForceSixPose = false;
    cfg.Calibration.ForceStatic = false;
    cfg.Calibration.SixPoseSecondsPerPose = 2.0;
    cfg.Calibration.GyroBiasSeconds = 1.5;
    cfg.Calibration.ForceDirection = false;
    cfg.Calibration.DirectionMinimumPeakRadPerSec = 0.30;

    cfg.Kick.DownstrokeArmRadPerSec = 0.50;
    cfg.Kick.BottomReversalRadPerSec = -0.10;
    cfg.Kick.AccelStepThresholdMps2 = 2.5;
    cfg.Kick.MissedHitResetRadPerSec = -0.60;
    cfg.Kick.MinReturnTravelRad = deg2rad(8);
    cfg.Kick.MinInterHitSeconds = 0.08;
    cfg.Kick.AudioVoices = 8;

    cfg.Orientation.ReferenceFrame = "ENU";
    cfg.Orientation.WarmupSeconds = 2.0;
    cfg.Orientation.PlotEverySamples = 5;
    cfg.Orientation.MaximumPlotPoints = 1200;

    cfg.Logging.BufferSamples = 100;

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
