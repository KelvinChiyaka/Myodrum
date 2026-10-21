# MyoDrum project structure

```text
Myodrum/
|-- mainMyoDrum.m
|-- IMU_subsystem/
|-- EMG_subsystem/
|-- calibration/
|   `-- legacy/
|-- classifiers/
|   |-- ruleBased/
|   `-- machineLearning/
|-- drumRunTests/
|   |-- kickOnly/
|   |-- pilotMapping/
|   `-- machineLearning/
|-- common/
|-- data/
|   |-- raw/
|   |-- processed/
|   |-- metadata/
|   `-- models/
|-- audio/
|-- tests/
|-- docs/
|-- MyoDrum.prj
|-- README.md
`-- .gitignore
```

## Responsibilities

- `mainMyoDrum.m` is the small integration and session entry point. Its startup
  menu selects IMU only, EMG only, four-channel IMU + EMG, or the left-stick
  three-EMG-channel profile (six sensing electrodes), followed by the classifier
  choice. Rule-based, trained bagged-tree ML and conservative hybrid
  classification remain available for the four-channel path. The left-stick
  profile supports rule-based operation immediately and uses its own labelled
  dataset and trained ML model. The entry point selects modes but does not
  duplicate sensor or classifier algorithms.
- `IMU_subsystem/` owns MPU6050 connection, register configuration, acquisition,
  bias/scale calibration, health checks, and IMU-derived motion features.
- `EMG_subsystem/` will own EMG acquisition, channel mapping, filtering,
  baselines, normalization, quality checks, and activation features.
- `calibration/` will coordinate session calibration and discrete target
  calibration, including the separate machine-learning dataset collection
  entry point. Sensor calibration and target calibration remain distinct.
- `classifiers/ruleBased/` contains explicit, interpretable thresholds and
  decision logic independent of audio playback.
- `classifiers/machineLearning/` contains feature preparation, grouped
  validation, training, inference, and model metadata.
- `drumRunTests/` contains laboratory workflows that call reusable code. The
  known working kick prototype and earlier five-drum pilot are retained here.
  The machine-learning collector implementation records balanced, labelled
  synchronized IMU/EMG blocks and is launched by
  `calibration/calibrateMachineLearningDataset.m`. The separate
  `calibration/calibrateLeftArmMachineLearningDataset.m` wrapper records the
  left-stick three-channel experiment without mixing it into the completed
  four-channel dataset.
- `common/` contains shared configuration, labels, validation, timestamps, and
  logging helpers.
- `data/` is the runtime data boundary. Raw trials are immutable; derived data
  and trained models go into separate folders and are ignored by Git by default.
- `audio/` contains tracked drum samples and their eventual label mapping.
- `tests/` contains hardware-independent MATLAB tests.
- `docs/` contains calibration, dataset, experiment, and troubleshooting
  documentation.

## Preserved experimental history

The five-drum/two-IMU mapping program is retained in
`drumRunTests/pilotMapping`. Its cross-round results (36.4% left and 25.8%
right) show that those features were not stable enough for live five-class
classification. It is evidence and a starting point, not the active model.

The active pilot therefore uses three targets and will first collect a small
session-aware dataset before scaling up.

## Current integrated path

`mainMyoDrum("imu-only", "rule-based")` coordinates these layers:

1. `IMU_subsystem/` connects and reads both sensors.
2. `calibration/calibrateDualIMUSession.m` loads/runs six-pose calibration,
   measures session gyro bias, and establishes each downstroke direction.
3. Two Sensor Fusion Toolbox `imufilter` objects estimate orientation.
4. `classifiers/ruleBased/` runs one independent robust kick detector per hand.
5. `audio/` supplies a preloaded polyphonic kick player bank.
6. `common/DualIMUSessionLogger.m` writes paired samples and event flags into
   the correct session-specific data folders.
