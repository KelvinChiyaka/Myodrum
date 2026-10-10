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

## Standalone EMG envelope sessions

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
