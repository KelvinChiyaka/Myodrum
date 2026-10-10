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
