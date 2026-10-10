# Repository audit

Audit date: 2026-10-10

## Original repository state

The `main` branch contained:

- six MATLAB files under `IMU_subsystem/`;
- dual-MPU6050 detection and reads for addresses `0x68` and `0x69`;
- static gyro calibration, six-pose accelerometer calibration, a long gyro
  drift experiment, and a dual-IMU plotting script;
- five WAV files under `Audio/`;
- MATLAB Project metadata;
- no top-level README or integration entry point;
- no EMG subsystem, shared configuration, classifier folders, session/target
  calibration layer, dataset boundary, tests, or experimental documentation.

## Important findings

1. The repository's IMU functions and the known working kick prototype use
   different acquisition implementations. They should not be merged until both
   are tested on the physical hardware.
2. `mainIMU.m` runs a static calibration and then a full six-pose calibration
   every time. That is suitable for a calibration experiment but too slow for
   an ordinary playing session.
3. `sixPoseCalibration.m` lacks a gravity/pose plausibility check, so an
   incorrectly placed pose can produce a formally saved but invalid result.
4. `staticCalibration.m` applies its movement threshold to disconnected rows as
   well as connected sensors. This is harmless when both are connected but
   should be corrected during the acquisition refactor.
5. The earlier five-drum mapping features had poor held-out-round accuracy.
   Randomly mixing hits across rounds would overstate expected live performance.

## Changes made in the architecture pass

- Added the agreed subsystem, classifier, calibration, experiment, data, test,
  and documentation folders.
- Added `mainMyoDrum.m` as a small controller and `common/myoDrumConfig.m` as the
  first shared configuration source.
- Preserved the working kick detector state machine, thresholds, and duplicate-
  hit safeguard while adding saved accelerometer calibration around it.
- Preserved the five-drum pilot and its analysis as research history.
- Set the active pilot targets to kick, snare, and crash.
- Renamed `Audio/` to `audio/` to match the agreed structure.
- Added hardware-independent structure/configuration tests and protected runtime
  data/calibration outputs in `.gitignore`.

## Deliberately not changed yet

- Existing IMU register and calibration functions were not rewritten without a
  hardware test.
- The working kick detector's thresholds and state machine were not refactored.
- No EMG interface, target classifier, or claimed accuracy was invented.
- No generated dataset or trained model was added to source control.
