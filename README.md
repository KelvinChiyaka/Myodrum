# MyoDrum

MyoDrum is a MATLAB virtual-drum research project using two MPU6050 inertial
measurement units (one per hand) and optional EMG sensing. The investigation
will compare rule-based and machine-learning classification using IMU-only and
IMU+EMG inputs.

The repository is organised around reproducible calibration and labelled data
collection. The immediate pilot uses three targets: **kick, snare, and crash**.

## Current working baseline

The restored one-IMU kick detector is preserved as the behavioural baseline in
`drumRunTests/kickOnly/mainKickIMUFast.m`. It detects a downstroke followed by
bottom reversal and acceleration jerk. Its return-travel state and minimum
inter-hit interval prevent one physical stroke from generating several sounds.

The integrated version adds six-pose accelerometer calibration and static gyro
calibration around that detector. Both results are saved and reused unless a
force-recalibration flag is enabled. Calibrated acceleration is used by the
jerk detector without changing its state machine.

From the repository root in MATLAB:

```matlab
mainMyoDrum("kick-only")
```

For the current two-hand workflow, use:

```matlab
mainMyoDrum("dual-kick")
```

This mode connects to both MPU6050s, loads or performs six-pose calibration
for each sensor, measures fresh static gyro bias for both sticks, estimates
tilt and relative azimuth with one `GravityAzimuthEstimator` per IMU, plays
the kick sample from either hand, and records the paired stream. Raw session
data are written to `data/raw/session_<timestamp>/`; session metadata and gyro
calibration are written to `data/metadata/sessions/session_<timestamp>/`.
Persistent six-pose and downstroke-direction calibrations live in
`data/metadata/calibration/`.

The MPU6050 has no magnetometer, so heading (azimuth) has no absolute
reference and drifts. Tilt (pitch/roll) comes from gravity and does not.
`common/GravityAzimuthEstimator.m` therefore:

- runs on every sample with the real timestamps (not a startup-measured rate);
- corrects tilt from the accelerometer only while the reading is about 1 g and
  the stick is not swinging, so strokes and impacts do not pollute it;
- leaves the stroke axis out of the azimuth integral, so a fast downstroke
  cannot leak into the heading;
- re-estimates residual gyro bias and freezes azimuth while the stick is still.

In three-drum mode either stick can play kick, snare, or crash:

```matlab
mainMyoDrum("three-drum")
```

The default `gravity-zones` strategy calibrates three independent 3-D gravity
directions for each stick. It compares unit vectors directly rather than
Euler pitch/roll, avoiding roll wrap and the pitch +/-90 degree singularity.
During calibration, aim or tilt the prompted stick
toward the printed drum and keep that pose consistent across its strokes.
Horizontal translation without a change in stick tilt is not observable by an
MPU6050. Calibration is repeated when the regions overlap or self-accuracy is
below 80%, and an unreliable map is never allowed into live play.

The calibration also learns a peak gyroscope direction for every hand/drum.
This is a secondary safeguard: an inconsistent swing is rejected but can
never turn one drum label into another. Acceleration jerk remains responsible
for confirming the physical hit.

While playing, the closest unambiguous gravity region is selected continuously and
latched when the downstroke arms. Impact acceleration therefore cannot change
the sound halfway through a stroke. A wider exit boundary provides hysteresis;
orientations outside every calibrated region are silent. Region models are
saved in `data/metadata/calibration/threeDrumGravityRegionModel.mat` and reused
until `cfg.Calibration.ForceRegionCalibration` is set to `true`.

The older `azimuth` and `hand-posture` strategies remain available for
experiments, but neither is the default.

The gyroscope range defaults to +/-1000 dps (`cfg.IMU.GyroscopeRangeDps`).
At 250 dps a fast stroke or a sweep between drums clips and the lost rotation
corrupts the integrated heading. If kick detection feels different, set it
back to 250 (positions will then be less accurate) and tell me.

Live orientation graphs are disabled in playing modes because rendering them
caused large audio delays; orientation is still estimated and recorded.

Orientation can be re-evaluated offline from any recorded session:

```matlab
addpath tools
orientationReplayReport("session_20261010_150626")
```

The legacy dual-IMU diagnostic and calibration program remains available:

```matlab
mainMyoDrum("imu-diagnostics")
```

Use `mainMyoDrum("status")` to display the shared configuration.

To record the four standalone EMG envelope channels without IMUs, audio, or
hit detection, run:

```matlab
mainMyoDrum("emg")
```

The recorder saves raw voltages and session metadata under the same timestamped
`data/` convention as the IMU recorder and reports its measured set rate.

For hit-timing data, upload the tracked Mega firmware under
`firmware/MyoDrumEMGStream/` and use `mainMyoDrum("emg-stream")`. This replaces
four slow `readVoltage` round trips with a continuous 250 Hz binary stream.

For labelled, synchronized gesture recordings from both IMUs and all four EMG
envelopes, run `mainMyoDrum("imu-emg")`. This mode repeats the three-drum
calibration and records host acquisition-window timestamps. It is intended for
dataset collection; the ordinary `three-drum` mode remains faster for playing.

For real-time synchronized fusion, upload
`firmware/MyoDrumIMUEMGStream/MyoDrumIMUEMGStream.ino` to the Mega and run:

```matlab
mainMyoDrum("imu-emg-stream")
```

This firmware puts both MPU6050 bursts and all four EMG envelopes in the same
200 Hz device-timestamped packet. The IMU chooses the drum; EMG only confirms
the strike. A rest/active calibration normalizes every EMG channel separately,
so independently adjusted amplifier gains do not share a voltage threshold.

To deliberately repeat calibration, edit `common/myoDrumConfig.m`:

```matlab
cfg.Calibration.ForceStatic = true;
cfg.Calibration.ForceSixPose = true;
cfg.Calibration.ForceDirection = true;
```

Return the flags to `false` after new results have been saved. With all three
flags false, valid cached calibration is loaded and the program proceeds
automatically to orientation warm-up and kick playback.

## Hardware assumed by the current code

- Arduino Mega 2560 on `COM7` (change this in `common/myoDrumConfig.m` and in
  the preserved prototype while it still has local settings).
- Left/right MPU6050 devices at I2C addresses `0x68` and `0x69`.
- MATLAB Support Package for Arduino Hardware.
- The ML analysis experiment additionally needs Statistics and Machine
  Learning Toolbox.

## Development sequence

1. Verify both IMUs, their hand assignment, measured read rate, and data quality.
2. Implement guided per-session calibration for kick, snare, and crash.
3. Record labelled raw trials and metadata without overwriting earlier sessions.
4. Establish a rule-based baseline.
5. Train and evaluate ML models with session/round-aware splits.
6. Add optional synchronized EMG and compare IMU-only with IMU+EMG.
7. Integrate verified components through `mainMyoDrum.m`.

See `PROJECT_STRUCTURE.md` for folder responsibilities and `docs/` for the
initial experimental protocols.
