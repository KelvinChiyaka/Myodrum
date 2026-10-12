# Initial dataset schema

Each session requires a unique session identifier and a manifest containing the
software revision, sensor addresses, hand assignment, nominal and measured
sample rates, units, calibration results, target layout, and warnings.

Each recorded trial should contain:

- session and trial identifiers;
- target label and hand label;
- trial type: calibration, training, validation, or held-out test;
- synchronized timestamps, accelerometer, gyroscope, and optional EMG samples;
- event timestamp and trial boundaries;
- validity flag, exclusion reason, and operator notes;
- preprocessing/feature version for derived outputs.

Raw time-series data remain immutable. Processed signals, features, and trained
models are saved separately. CSV may be used for trial metadata or feature
tables, while synchronized multichannel time series should use a documented MAT
structure rather than ambiguous flattened columns.

The current paired live stream additionally records:

- `frameValid` (`N`-by-2 logical): whether each hand supplied a fresh frame;
- `loopDuration_s` (`N`-by-1 seconds): complete acquisition-loop timing;
- `targetIndex` (`N`-by-2 uint8): zero for no accepted target, otherwise an
  index into `metadata.config.Targets.Names`;
- `kickHit` (`N`-by-2 logical): the historical field name for the per-hand
  strike-candidate flag; in three-drum mode it can accompany any target.

## Standalone EMG envelope sessions

## Combined IMU and EMG sessions

Run `mainMyoDrum("imu-emg")` to create a fresh labelled three-drum
calibration followed by live play. The raw session folder contains:

- `imuEmgGestureCalibration.mat`: continuous two-IMU and four-channel EMG
  samples labelled by `handIndex`, `targetIndex`, and calibration `attempt`;
- `dualKickIMU.mat`: live two-IMU data, detected hits and target indices, plus
  `emgVoltage_V` in `cfg.EMG.ChannelNames` order.

Both files store `imuReadStart_s`, `imuReadEnd_s`, `imuTimestamp_s`,
`emgReadStart_s`, `emgReadEnd_s`, and `emgTimestamp_s`. Midpoint timestamps
are derived from each host acquisition window. They share one MATLAB
`tic`/`toc` clock but are not hardware-clock timestamps. Four EMG channels are
read sequentially, so the window endpoints expose their timing uncertainty.

This compatibility mode uses the MATLAB Arduino Support Package. It cannot
run at the standalone 250 Hz EMG firmware rate because both methods require
exclusive ownership of the same COM port.

## Real-time synchronized fusion sessions

`mainMyoDrum("imu-emg-stream")` writes `imuEmgStream.mat`. Every row is one
200 Hz firmware frame and contains `time_s`, `sequenceNumber`, `deviceStatus`,
`frameValid`, both IMUs, `emgVoltage_V`, independently normalized
`emgNormalized`, `imuCandidateHit`, `emgConfirmedHit`,
`emgConfirmationScore`, and `targetIndex`. All values in a row share the same
Arduino `micros()` timestamp. `emgConfirmationCalibration.mat` stores the rest
and active samples plus the per-channel normalization profile.

The continuous file also includes `candidateTargetIndex`,
`fusionAcceptedHit`, `fusionRejectedHit`, `fusionPendingActive`, and
`fusionDecisionCode`. The `fusionEvents` table contains one row per IMU
candidate with candidate and decision timestamps, hand, proposed orientation
label, initial and peak normalized EMG scores, acceptance, and decision reason.
Code meanings are stored in session metadata. A recoverable checkpoint is
written every `cfg.Logging.StreamCheckpointSeconds`; only the final save marks
the session `completedNormally=true`.

`mainMyoDrum("emg")` writes `emgAcquisition.mat` beside the IMU session folders.
Its variables are:

- `initialSessionId`: session identifier stored when the incremental MAT file
  is created;
- `time_s` (`N`-by-1 double): host-time midpoint of each fixed-order channel
  set;
- `emgVoltage_V` (`N`-by-`C` double): unfiltered hardware-envelope voltage in
  `metadata.channelNames` order;
- `setDuration_s` (`N`-by-1 double): first-to-last read duration for a set;
- `phase` (`N`-by-1 uint8): 1 REST, 2 FREE;
- `marker` (`N`-by-1 uint8): 0 none, 1 intended hit, 2 unintended motion,
  3 maximum comfortable reference squeeze.

The timestamp is produced by host `tic`/`toc`, not the sensor device. Because
each `readVoltage` call is a serial round trip, the measured set rate and any
undersampling warning must be interpreted with every recording. Voltage gains
are channel- and session-specific; raw amplitudes must not be compared without
the saved gain notes and an appropriate per-channel normalization procedure.

High-rate `emg-stream` sessions use the same file and phase/marker variables.
They additionally store `sequenceNumber` (`N`-by-1 uint16) and `deviceStatus`
(`N`-by-1 uint8). Their `time_s` is derived from the rollover-corrected Arduino
`micros()` clock; `metadata.streamStatistics` records dropped frames, checksum
failures, resynchronisations and measured device rate.

## Machine-learning labelled collection

Run `calibrateMachineLearningDataset` from the `calibration/` workflow with the
synchronized dual-IMU/four-channel firmware. Each `mlTrainingStream.mat` row
contains the two raw and calibrated
accelerometers, raw and bias-corrected gyroscopes, temperatures, raw ADC values,
EMG volts, normalized EMG, device timestamp, validity, block, round, prompted
hand, and prompted class index. `classNames` maps class indices to `NO_HIT`,
`KICK`, `SNARE`, and `CRASH`.

`blockManifest` and `mlCollectionManifest.csv` store one row per prompted
block, including the round, hand, class, expected hit count, actual recording
duration, frame count, and completion status. A block label is interval-level
ground truth: inter-strike frames in a hit block are not themselves hits.
Feature preparation must segment candidate movement windows inside the block
before training. Training, validation, and test partitions must remain grouped
by session or collection round.
