# EMG subsystem

This folder contains standalone EMG envelope acquisition, robust rest-baseline,
and signal-quality helpers. Start a recording from the repository root with:

```matlab
mainMyoDrum("emg")
```

The Mega 2560 channel map is A0 left forearm, A2 left triceps, A3 right
forearm, and A4 right triceps. A1 and A5 are reserved for future biceps
channels. Samples are unfiltered 0-5 V hardware-envelope readings; the code
does not compare absolute amplitudes across independently adjusted channels.

`cfg.EMG.Enabled` remains false because it controls future IMU+EMG fusion, not
this standalone recorder. No synthetic EMG values are used at runtime.
