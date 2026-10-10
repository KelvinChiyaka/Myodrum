# Standalone EMG recording

Run `mainMyoDrum("emg")` from the project root. The recorder does not connect
to either IMU, play audio, detect hits, or classify motions.

After the rest baseline, use these keys during free recording:

- `H`: mark the next set as an intended hit;
- `X`: mark the next set as an unintended or false motion;
- `R`: mark the next set as a maximum comfortable reference squeeze;
- `Q`: stop and save.

Each channel has an independent gain control. Record the gain-pot positions by
calling `mainEMGRecord(GainNotes=[...])` directly when that information is
available. The measured acquisition rate is saved and printed because MATLAB
`readVoltage` performs a separate serial transaction for every channel.
