# Calibration

Calibration has two separate jobs:

1. Sensor calibration estimates MPU6050 gyro bias and accelerometer offset/scale.
2. Target calibration records labelled examples for the configured virtual
   drums (currently kick, snare, and crash).

Target calibration will run once per session because stick mounting, posture,
sensor bias, and the physical target layout may change. Calibration recordings
must be labelled separately from training and held-out evaluation trials.

The earlier five-drum mapping experiment is preserved under
`drumRunTests/pilotMapping`; it is not the active three-drum calibration.

`calibrateDualIMUSession.m` is the active two-hand calibration coordinator:

- each IMU has its own saved six-pose accelerometer offset/scale result;
- both IMUs receive a fresh static gyro-bias measurement every session;
- each stick has a saved downstroke-axis/sign calibration;
- reusable mounting calibration is stored under `data/metadata/calibration/`;
- session-specific gyro results are stored with that session's metadata.
