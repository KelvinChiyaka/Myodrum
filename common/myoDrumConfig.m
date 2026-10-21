function cfg = myoDrumConfig(projectRoot, sensorProfile)
% MYODRUMCONFIG Return the shared MyoDrum hardware and experiment settings.
%
% Keep settings used by more than one subsystem here. Hardware-specific
% tuning that belongs to a single detector may remain beside that detector
% until it is experimentally validated and promoted into the shared pipeline.

    if nargin < 1 || strlength(string(projectRoot)) == 0
        projectRoot = fileparts(fileparts(mfilename('fullpath')));
    end
    if nargin < 2 || strlength(string(sensorProfile)) == 0
        sensorProfile = "four-channel";
    end
    projectRoot = char(projectRoot);

    cfg.Project.Name = "MyoDrum";
    cfg.Project.DisplayName = "MyoDrum: EMG and IMU Virtual Drum Instrument";
    cfg.Project.SchemaVersion = "0.1.0";

    cfg.Hardware.COMPort = "COM7";
    cfg.Hardware.Board = "Mega2560";

    cfg.IMU.Addresses = ["0x68", "0x69"];
    cfg.IMU.HandNames = ["LEFT", "RIGHT"];
    cfg.IMU.ActiveHandIndices = [1, 2];
    % A faster internal update and lower-latency DLPF ensure each slow
    % MATLAB/I2C transaction receives the freshest available motion frame.
    cfg.IMU.SensorOutputRateHz = 100;
    cfg.IMU.DLPFConfig = 2;
    cfg.IMU.AccelerometerRangeG = 2;
    % +/-250 dps clips a fast stroke or a sweep between drums (about 80% of
    % hit windows in the recorded sessions), and clipped rotation is lost
    % from the integrated heading. 1000 dps keeps sweeps unclipped; the
    % MPU6050 noise density is the same at every range.
    cfg.IMU.GyroscopeRangeDps = 1000;
    % The binary stream reports each failed I2C transaction. Allow a small
    % number of transient misses while still rejecting a genuinely unstable
    % or disconnected sensor before audio starts.
    cfg.IMU.StreamStartupSettleSeconds = 0.25;
    cfg.IMU.StreamWarningValidFraction = 0.99;
    cfg.IMU.StreamMinimumValidFraction = 0.95;

    cfg.Calibration.ForceSixPose = false;
    cfg.Calibration.ForceStatic = false;
    cfg.Calibration.SixPoseSecondsPerPose = 2.0;
    cfg.Calibration.GyroBiasSeconds = 1.5;
    cfg.Calibration.GyroBiasPolicy = "fresh";
    cfg.Calibration.GyroBiasMaxAgeHours = 8;
    cfg.Calibration.ForceDirection = false;
    cfg.Calibration.ForceRegionCalibration = false;
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
    cfg.Kick.ArmTimeoutSeconds = 1.0;

    cfg.Orientation.ReferenceFrame = "ENU";
    cfg.Orientation.WarmupSeconds = 2.0;
    % Hit detection runs every acquisition. Orientation and drawing are
    % deliberately decimated because they are not in the audio trigger path.
    cfg.Orientation.UpdateEverySamples = 4;
    % MATLAB graphics caused 0.3-0.4 s stalls during playing. Keep the
    % diagnostic plot available, but refresh it only occasionally.
    cfg.Orientation.PlotEverySamples = 1000;
    cfg.Orientation.ControlPollEverySamples = 10;
    cfg.Orientation.MaximumPlotPoints = 1200;
    % Tilt/azimuth estimator (common/GravityAzimuthEstimator.m). It runs on
    % every sample with the real timestamps. Leave empty to use its defaults;
    % set fields (for example GravityGainPerSec) to override them.
    cfg.Orientation.EstimatorOverrides = struct();
    cfg.Orientation.MaxWarmupSeconds = 12;
    cfg.Orientation.EstimatorVersion = "gravity-azimuth-1";

    % Live samples are stored in a geometrically growing preallocated buffer.
    % Saving the complete and continually growing session during play blocks
    % the 200 Hz loop, so checkpoints are opt-in. The full session is always
    % saved when the user stops normally.
    % 120000 rows covers ten minutes at 200 Hz without even one resize.
    cfg.Logging.BufferSamples = 120000;
    cfg.Logging.StreamCheckpointSeconds = inf;

    % Keep plotting out of the live-play path. The dashboard helpers remain
    % available for offline diagnostics, but MATLAB graphics must not compete
    % with the 200 Hz detector, fusion logic or audio playback.
    cfg.Visualization.Enabled = false;
    cfg.Visualization.RefreshRateHz = 5;
    cfg.Visualization.WindowSeconds = 6;
    cfg.Visualization.MaximumPoints = 600;
    % Print only completed fusion decisions, rather than continuous sensor
    % values. This gives immediate, low-volume feedback without live plots.
    cfg.Runtime.PrintHitEvents = true;

    % The first controlled pilot deliberately uses three separable targets.
    cfg.Targets.Names = ["KICK", "SNARE", "CRASH"];
    cfg.Targets.CalibrationHitsPerTargetPerHand = 10;
    cfg.Targets.AudioFiles = ["kick.wav", "snare1.wav", "ridecrash.wav"];

    cfg.Regions.MinimumSpreadDeg = [5, 5];
    cfg.Regions.MaximumNormalizedDistance = 3.0;
    cfg.Regions.AmbiguityMargin = 0.35;

    % Three-drum strategy. In "gravity-zones" both sticks can select every
    % drum from calibrated pitch/roll poses. Pitch and roll are referenced
    % to gravity, so the layout cannot rotate around the player like yaw.
    % "azimuth": any stick plays the drum it is aimed at (left/front/right),
    % using heading with drift tracking. "hand-posture": hand decides the
    % ordinary sound, a gravity posture plays crash (drift-free fallback).
    cfg.Regions.Strategy = "gravity-zones";
    cfg.Regions.GravityZoneCalibrationAttempts = 2;
    cfg.Regions.GravityZoneMinAccuracy = 0.80;
    cfg.Regions.GravityZoneMinSeparationDeg = 20;
    cfg.Regions.GravityZoneMinSpreadDeg = 6;
    cfg.Regions.GravityZoneMinGyroSpreadDeg = 12;
    cfg.Regions.GravityZoneMaximumGyroAngleDeg = 80;
    cfg.Regions.GravityZoneExitDistance = 3.75;
    cfg.Regions.CalibrationOrder = ["SNARE", "KICK", "CRASH", "KICK"];
    cfg.Regions.CalibrationRounds = 6;
    % Heading used for a hit: "hit" (at detection) or "arm" (when the
    % downstroke armed). "hit" was accurate in simulation; "arm" could be
    % stale when the stick was still moving to the next drum.
    cfg.Regions.AzimuthFeatureTime = "hit";
    cfg.Regions.AzimuthMinSpreadDeg = 6;
    cfg.Regions.AzimuthMinSeparationDeg = 15;
    cfg.Regions.AzimuthMinAccuracy = 0.80;
    cfg.Regions.AzimuthCalibrationAttempts = 2;
    cfg.Regions.VerifyHitsPerHand = 12;
    cfg.Regions.VerifyMinAccuracy = 0.75;
    cfg.Regions.Tracker = struct();      % overrides for DrumAzimuthTracker
    cfg.Regions.HandDrums = ["SNARE", "KICK"];   % LEFT, RIGHT
    cfg.Regions.PostureTarget = "CRASH";
    cfg.Regions.PostureFeatureTime = "arm";      % "arm" or "hit"
    cfg.Regions.PostureMinSpreadDeg = 4;
    cfg.Regions.PostureMinSeparationDeg = 20;
    cfg.Regions.PostureMinAccuracy = 0.85;
    cfg.Regions.PostureMaxNormalizedDistance = 3.0;
    cfg.Regions.PostureMarginNormalized = 0.25;
    cfg.Regions.PostureCalibrationAttempts = 3;

    % EMG voltages are rectified analog envelope outputs in the 0-5 V range.
    cfg.EMG.Enabled = false;
    cfg.EMG.AnalogPins = ["A0"; "A2"; "A3"; "A4"];
    cfg.EMG.ChannelNames = ["LEFT_FOREARM"; "LEFT_TRICEPS"; ...
        "RIGHT_FOREARM"; "RIGHT_TRICEPS"];
    cfg.EMG.Hands = ["LEFT"; "LEFT"; "RIGHT"; "RIGHT"];
    cfg.EMG.ActiveChannelMask = true(4,1);
    cfg.EMG.RestSeconds = 5;
    cfg.EMG.PreflightSeconds = 2;
    cfg.EMG.LiveWindowSeconds = 15;
    cfg.EMG.PlotEverySets = 3;
    cfg.EMG.MinRecommendedSetRateHz = 50;
    cfg.EMG.Quality.FlatRangeVolts = 0.005;
    cfg.EMG.Quality.RailVolts = 4.8;
    cfg.EMG.Quality.RestSigmaWarnVolts = 0.15;
    % Combined play fails closed when floating/disconnected analog inputs
    % rail, remain noisy at rest, or cannot rise clearly above their noise.
    cfg.EMG.Quality.MaximumRailFraction = 0.02;
    cfg.EMG.Quality.MinimumSignalToNoiseRatio = 4.0;
    cfg.EMG.Quality.RuntimeRailHoldSeconds = 1.0;
    cfg.EMG.StreamBaud = 250000;
    cfg.EMG.StreamRateHz = 250;
    cfg.EMG.StreamProtocolVersion = 1;
    cfg.EMG.ADCReferenceVolts = 5.0;
    cfg.EMG.ADCMaxCount = 1023;
    cfg.EMG.Quality.NoiseFloorVolts = ...
        cfg.EMG.ADCReferenceVolts / cfg.EMG.ADCMaxCount;
    cfg.EMG.StreamPlotEverySeconds = 0.10;
    cfg.EMG.HitThresholdVolts = 3.0; % Placeholder for later EMG fusion.
    % Synchronized stream/fusion settings. Every channel is normalized using
    % its own rest level and active reference, so amplifier gain differences
    % do not create unequal confirmation thresholds.
    cfg.EMG.CombinedStreamRateHz = 200;
    cfg.EMG.CombinedStreamBaud = 250000;
    cfg.EMG.StreamTimeoutSeconds = 3.0;
    cfg.EMG.ActiveCalibrationSeconds = 6;
    cfg.EMG.ConfirmationWindowSeconds = 0.18;
    cfg.EMG.NormalizedConfirmationThreshold = 0.20;
    cfg.EMG.MinimumActiveSpanVolts = 0.10;

    % Rule-based IMU + EMG fusion. The IMU detects and labels candidates;
    % normalized EMG only confirms them. Timers are independent per hand.
    cfg.Fusion.EMGThresholdNormalized = 0.20;
    cfg.Fusion.EMGLookbackSeconds = 0.18;
    cfg.Fusion.EMGConfirmationTimeoutSeconds = 0.12;
    cfg.Fusion.MinimumInterAcceptedHitSeconds = 0.28;
    % Require agreement from both same-hand electrodes. A single floating,
    % loose or motion-contaminated lead must never authorize an IMU hit.
    cfg.Fusion.RequiredActiveChannels = 2;

    % Labelled collection for the first standalone machine-learning model.
    % Each round contains every hand/class combination. Hit blocks are ended
    % by the operator after the requested repetitions; NO_HIT blocks enforce
    % a minimum duration so normal movement is adequately represented.
    cfg.MachineLearning.ClassNames = ["NO_HIT", cfg.Targets.Names];
    cfg.MachineLearning.CollectionRounds = 3;
    cfg.MachineLearning.HitsPerClassPerHandPerRound = 20;
    cfg.MachineLearning.NoHitMinimumSeconds = 10;
    cfg.MachineLearning.ScheduleVersion = "ml-blocks-1";
    cfg.MachineLearning.FeatureVersion = "imu-emg-window-1";
    cfg.MachineLearning.FeatureWindowPreSeconds = 0.20;
    cfg.MachineLearning.FeatureWindowPostSeconds = 0.10;
    cfg.MachineLearning.FeatureMinimumSeparationSeconds = 0.30;
    cfg.MachineLearning.NoHitWindowsPerBlock = 20;
    cfg.MachineLearning.BaggedTreeCount = 150;
    cfg.MachineLearning.BaggedTreeMinLeafSize = 5;
    cfg.MachineLearning.BaggedTreeMaxSplits = 100;
    cfg.MachineLearning.ModelVersion = "bagged-trees-1";
    cfg.MachineLearning.DataFolderName = "machineLearning";
    cfg.MachineLearning.ModelFolderName = "machineLearning";
    cfg.MachineLearning.ModelFileName = "myoDrumBaggedTrees.mat";
    % Live ML evaluates only IMU strike candidates. The model chooses the
    % drum (or NO_HIT), while two healthy same-hand EMG channels remain a
    % mandatory safety gate. These settings do not alter rule-based play.
    cfg.MachineLearning.LiveMinimumConfidence = 0.50;
    cfg.MachineLearning.LiveMinimumWindowFraction = 0.80;
    cfg.MachineLearning.LiveBufferSeconds = 1.0;
    cfg.MachineLearning.RequireEMGConfirmation = true;

    cfg.Paths.ProjectRoot = string(projectRoot);
    cfg.Paths.Audio = string(fullfile(projectRoot, 'audio'));
    cfg.Paths.DataRoot = string(fullfile(projectRoot, 'data'));
    cfg.Paths.RawData = string(fullfile(projectRoot, 'data', 'raw'));
    cfg.Paths.ProcessedData = string(fullfile(projectRoot, 'data', 'processed'));
    cfg.Paths.Metadata = string(fullfile(projectRoot, 'data', 'metadata'));
    cfg.Paths.Models = string(fullfile(projectRoot, 'data', 'models'));

    sensorProfile = lower(strtrim(string(sensorProfile)));
    switch sensorProfile
        case {"four-channel", "four", "dual"}
            cfg.Runtime.SensorProfile = "four-channel";
        case {"left-three-emg", "left-only", "six-electrode"}
            % Three bipolar EMG channels use six sensing electrodes on the
            % left arm. The fourth ADC position remains in the established
            % binary packet so the proven 200 Hz firmware does not change.
            cfg.Runtime.SensorProfile = "left-three-emg";
            cfg.IMU.ActiveHandIndices = 1;
            cfg.EMG.AnalogPins = ["A0"; "A1"; "A2"; "A4"];
            cfg.EMG.ChannelNames = ["LEFT_BRACHIORADIALIS"; ...
                "LEFT_TRICEPS"; "LEFT_ECRL"; "UNUSED_A4"];
            cfg.EMG.Hands = ["LEFT"; "LEFT"; "LEFT"; "UNUSED"];
            cfg.EMG.ActiveChannelMask = [true; true; true; false];
            cfg.Fusion.RequiredActiveChannels = 2;
            cfg.MachineLearning.FeatureVersion = "imu-left3-emg-window-1";
            cfg.MachineLearning.ModelVersion = "left3-bagged-trees-1";
            cfg.MachineLearning.DataFolderName = "machineLearningLeft3";
            cfg.MachineLearning.ModelFolderName = "machineLearningLeft3";
            cfg.MachineLearning.ModelFileName = ...
                "myoDrumLeft3BaggedTrees.mat";
        otherwise
            error('MyoDrum:UnknownSensorProfile', ...
                'Unknown sensor profile "%s".', sensorProfile);
    end
end
