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

## High-rate mode

After uploading `firmware/MyoDrumEMGStream/MyoDrumEMGStream.ino` to the Mega,
close the Arduino Serial Monitor and run:

```matlab
mainMyoDrum("emg-stream")
```

This mode opens COM7 directly with `serialport`, receives timestamped binary
packets at a firmware target of 250 complete channel sets per second, and logs
sequence numbers so dropped packets are measurable. It cannot share the port
with a MATLAB `arduino` object or another recorder.
