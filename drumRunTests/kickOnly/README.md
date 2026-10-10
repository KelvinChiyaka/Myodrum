# Working kick-only baseline

`mainKickIMUFast.m` is the restored one-IMU implementation that successfully
played rapid kick sounds. It is intentionally preserved unchanged before the
shared two-IMU refactor.

The detector requires:

1. A downstroke that arms the detector.
2. A reversal plus acceleration step at the bottom.
3. Sufficient upward return travel before re-arming.
4. A minimum time between accepted hits.

Run it from the repository root with `mainMyoDrum("kick-only")`.

On the first run, the program guides the operator through six accelerometer
poses and saves the result under `data/metadata/`. Later runs load that result.
Set `cfg.ForceSixPoseCalibration = true` only after changing the sensor mounting
or when repeating a suspect calibration. A short stationary gyro-bias
calibration still runs at every startup because gyro bias varies with temperature.
