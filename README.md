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

The integrated version now adds the missing six-pose accelerometer calibration
around that detector. The result is saved and reused; a fresh stationary gyro
bias (static calibration) is still measured at every startup. Calibrated
acceleration is used by the jerk detector without changing its state machine.

From the repository root in MATLAB:

```matlab
mainMyoDrum("kick-only")
```

For the current two-hand workflow, use:

```matlab
mainMyoDrum("dual-kick")
```

This mode connects to both MPU6050s, loads or performs six-pose calibration
for each sensor, measures a fresh static gyro bias for both sticks, estimates
relative yaw/pitch/roll with one `imufilter` per IMU, plays the kick sample from
either hand, and records the paired stream. Raw session data are written to
`data/raw/session_<timestamp>/`; session metadata and gyro calibration are
written to `data/metadata/sessions/session_<timestamp>/`. Persistent six-pose
and downstroke-direction calibrations live in `data/metadata/calibration/`.

The MPU6050 has no magnetometer. `imufilter` therefore gives useful
gravity-referenced roll and pitch, but yaw is only relative and will drift.

The legacy dual-IMU diagnostic and calibration program remains available:

```matlab
mainMyoDrum("imu-diagnostics")
```

Use `mainMyoDrum("status")` to display the shared configuration.

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
