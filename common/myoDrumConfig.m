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
    % A faster internal update and lower-latency DLPF ensure each slow
    % MATLAB/I2C transaction receives the freshest available motion frame.
    cfg.IMU.SensorOutputRateHz = 100;
    cfg.IMU.DLPFConfig = 2;
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
    cfg.Kick.DecelerationDropRadPerSec = 0.30;
    cfg.Kick.AccelStepThresholdMps2 = 2.5;
    cfg.Kick.MissedHitResetRadPerSec = -0.60;
    % Per-hand anti-double-trigger gate. Alternating and simultaneous hits
    % remain independent because each stick owns a separate detector.
    cfg.Kick.MinReturnTravelRad = deg2rad(15);
    cfg.Kick.MinInterHitSeconds = 0.28;
    cfg.Kick.AudioVoices = 8;

    cfg.Orientation.ReferenceFrame = "ENU";
    cfg.Orientation.WarmupSeconds = 2.0;
    % Hit detection runs every acquisition. Orientation and drawing are
    % deliberately decimated because they are not in the audio trigger path.
    cfg.Orientation.UpdateEverySamples = 4;
    cfg.Orientation.PlotEverySamples = 12;
    cfg.Orientation.MaximumPlotPoints = 1200;

    cfg.Logging.BufferSamples = 500;

    % The first controlled pilot deliberately uses three separable targets.
    cfg.Targets.Names = ["KICK", "SNARE", "CRASH"];
    cfg.Targets.CalibrationHitsPerTargetPerHand = 10;
    cfg.Targets.AudioFiles = ["kick.wav", "snare1.wav", "ridecrash.wav"];

    % EMG voltages are rectified analog envelope outputs in the 0-5 V range.
    cfg.EMG.Enabled = false;
    cfg.EMG.AnalogPins = ["A0"; "A2"; "A3"; "A4"];
    cfg.EMG.ChannelNames = ["LEFT_FOREARM"; "LEFT_TRICEPS"; ...
        "RIGHT_FOREARM"; "RIGHT_TRICEPS"];
    cfg.EMG.Hands = ["LEFT"; "LEFT"; "RIGHT"; "RIGHT"];
    cfg.EMG.RestSeconds = 5;
    cfg.EMG.PreflightSeconds = 2;
    cfg.EMG.LiveWindowSeconds = 15;
    cfg.EMG.PlotEverySets = 3;
    cfg.EMG.MinRecommendedSetRateHz = 50;
    cfg.EMG.Quality.FlatRangeVolts = 0.005;
    cfg.EMG.Quality.RailVolts = 4.8;
    cfg.EMG.Quality.RestSigmaWarnVolts = 0.15;
    cfg.EMG.HitThresholdVolts = 3.0; % Placeholder for later EMG fusion.

    cfg.Paths.ProjectRoot = string(projectRoot);
    cfg.Paths.Audio = string(fullfile(projectRoot, 'audio'));
    cfg.Paths.DataRoot = string(fullfile(projectRoot, 'data'));
    cfg.Paths.RawData = string(fullfile(projectRoot, 'data', 'raw'));
    cfg.Paths.ProcessedData = string(fullfile(projectRoot, 'data', 'processed'));
    cfg.Paths.Metadata = string(fullfile(projectRoot, 'data', 'metadata'));
    cfg.Paths.Models = string(fullfile(projectRoot, 'data', 'models'));
end
