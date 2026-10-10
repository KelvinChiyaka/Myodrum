# Troubleshooting

## No MPU6050 detected

- Confirm common ground, power, SDA, and SCL.
- Confirm AD0 is low for `0x68` and high for `0x69`.
- Close other MATLAB/Arduino connections to the same COM port and run `clear`.
- Verify `COM7` in the configuration matches the available port.

## Invalid or all-zero frame

Power-cycle the sensors, shorten/secure the I2C wiring, and rerun the diagnostic.
Do not convert an invalid frame into a zero-valued measurement.

## Duplicate kick sounds

The preserved kick detector requires return travel before re-arming. Confirm the
working baseline still contains `WAIT_RETURN_TRAVEL`, `MinReturnTravelRad`, and
`MinInterHitSeconds` before tuning thresholds.

## Poor target accuracy

Use held-out collection rounds. High random-fold accuracy with poor held-out-
round accuracy indicates drift or unstable features, not a reliable classifier.
Reduce the target set, verify hand/sensor orientation, and improve the captured
motion window before collecting a larger dataset.
