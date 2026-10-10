# Calibration protocol

## Session order

1. Record session ID, time, software revision, operator notes, and hardware map.
2. Confirm MPU6050 devices at `0x68` and `0x69` and verify which hand wears each.
3. Measure the achieved per-IMU read rate and check for invalid or repeated frames.
4. With both sticks still, estimate gyro bias and baseline noise independently.
5. Verify gravity magnitude and the expected sensor-axis convention.
6. Record guided target-calibration strikes for kick, snare, and crash.
7. Run known-target verification strikes before collecting experimental trials.

Start with 10 accepted calibration strikes per target per hand. This is a pilot
quantity, not a claimed optimum. Failed or ambiguous trials must be retried and
logged rather than silently accepted.

Sensor calibration values and target-calibration examples must be saved with the
session but identified separately from classifier training and evaluation data.
