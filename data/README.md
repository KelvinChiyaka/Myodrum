# Runtime data

- `raw/`: immutable synchronized sensor trials and session manifests.
- `processed/`: derived signals and feature tables.
- `metadata/`: schemas, exclusions, labels, and run summaries.
- `models/`: trained classifiers and their preprocessing/model metadata.

Generated contents are ignored by Git. Keep a small, deliberately anonymised
example dataset only when it is explicitly reviewed and added to the repository.

The active dual-kick mode creates matching timestamped session folders:

- `raw/session_<timestamp>/dualKickIMU.mat` contains timestamps, raw and
  calibrated acceleration, bias-corrected gyro, temperature, relative
  yaw/pitch/roll, and a two-column kick-event flag.
- `metadata/sessions/session_<timestamp>/` contains session configuration and
  the complete calibration used for that recording.
- `metadata/calibration/` contains reusable six-pose and direction calibration.

Standalone EMG recording uses the same session convention:

- `raw/session_<timestamp>/emgAcquisition.mat` contains `time_s`, the unfiltered
  multichannel `emgVoltage_V`, `setDuration_s`, `phase`, and `marker`;
- `metadata/sessions/session_<timestamp>/sessionMetadata.mat` records the pin
  map, channel names, gain notes, measured rate, baseline, and quality flags;
- `processed/session_<timestamp>/emgOverview.png` is a best-effort summary and
  is never required for the raw recording to be valid.
