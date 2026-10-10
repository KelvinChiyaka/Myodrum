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
|   `-- pilotMapping/
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

- `mainMyoDrum.m` is the small integration and session entry point. It selects
  modes but does not duplicate sensor or classifier algorithms.
- `IMU_subsystem/` owns MPU6050 connection, register configuration, acquisition,
  bias/scale calibration, health checks, and IMU-derived motion features.
- `EMG_subsystem/` will own EMG acquisition, channel mapping, filtering,
  baselines, normalization, quality checks, and activation features.
- `calibration/` will coordinate session calibration and discrete target
  calibration. Sensor calibration and target calibration remain distinct.
- `classifiers/ruleBased/` contains explicit, interpretable thresholds and
  decision logic independent of audio playback.
- `classifiers/machineLearning/` contains feature preparation, grouped
  validation, training, inference, and model metadata.
- `drumRunTests/` contains laboratory workflows that call reusable code. The
  known working kick prototype and earlier five-drum pilot are retained here.
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
