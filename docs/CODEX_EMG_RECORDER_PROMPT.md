# Codex task: EMG-only recording mode (four channels, standalone)

Audience: Codex (implementation). Claude wrote this brief; do not assume any of it is already implemented. Read the code you are told to mirror before writing anything, and verify field names and signatures yourself.

## 1. Goal

Add a standalone EMG recorder that runs **without the IMUs, audio or any detector**:

```matlab
mainMyoDrum("emg")
```

It must read the four EMG channels from the Mega 2560, show a light live plot, and save the data in the same place and with the same session conventions as the IMU recorder (`mainDualKickIMU`), so EMG sessions sit beside IMU sessions in `data/`.

This is data collection only. No gating, no hit detection, no classification, no fusion with IMU in this task.

## 2. Hardware facts (do not change)

- Board: Mega 2560 on `cfg.Hardware.COMPort` (`COM7`), `cfg.Hardware.Board`.
- Four analog **envelope** outputs (already rectified/smoothed in the sensor hardware, read as 0-5 V). Raw 1 kHz EMG is out of scope.
- Channel map (this is the partner's wiring, keep it):

| Column | Pin | Channel name | Hand |
|---|---|---|---|
| 1 | **A0** | `LEFT_FOREARM` | LEFT |
| 2 | A2 | `LEFT_TRICEPS` | LEFT |
| 3 | A3 | `RIGHT_FOREARM` | RIGHT |
| 4 | A4 | `RIGHT_TRICEPS` | RIGHT |

- The left forearm moved from A1 to **A0** after the first bench test. A1 is now unused. Any older script or note that says "A1 = left forearm" is out of date.
- A1 and A5 are reserved for the biceps later. Do not use them. Design everything so adding two channels later is a config change, not a rewrite (no hard-coded `4` or `1:4` outside the config).
- **Each sensor has its own adjustable gain pot, and the four are set differently.** Raw volts are therefore not comparable across channels, and not comparable across sessions if a pot was touched in between. The recorder must never compare channels in absolute volts, and it must record what the operator knows about the gain (see `GainNotes`, section 6).

### Observed behaviour on the bench (use this to set expectations, not as a spec)

- Rest is about 0.00 V on both right channels (the envelope clamps at zero when relaxed), about 0.02 V on the left forearm and about 0.15-0.20 V, slightly wandering, on the left triceps.
- Real contractions and strokes reach about 1-2.7 V. Nothing was seen reaching the 5 V rail.
- Measured by eye, sets arrive about every 0.3 s (3-4 sets/s). A burst is only 1-3 samples wide, so **recorded peak amplitudes are undersampled lower bounds** and burst timing is only good to a few hundred ms. This is expected with `readVoltage`. It must be reported honestly (see `MinRecommendedSetRateHz`), not hidden.
- A channel sitting at exactly 0 V while relaxed is normal and healthy. Do not treat it as a fault on its own (see 6.2 and 6.5).
- Acquisition uses the MATLAB Arduino support package: `arduino(port, board)`, `configurePin(a, pin, "AnalogInput")`, `readVoltage(a, pin)`. Do **not** pass `'Libraries','I2C'` (not needed) and do not create any I2C device.
- Known limit, accept and report honestly: every `readVoltage` is a serial round trip, so the set rate will be low (roughly tens of Hz at best, possibly a few Hz). Never claim or assume a rate. Measure and log it.

## 3. Ground rules

- MATLAB R2024b, Windows. Tests run with `tests/runAllTests.m`.
- Do NOT edit: `drumRunTests/kickOnly/mainKickIMUFast.m` (frozen baseline), anything under `IMU_subsystem/`, `classifiers/`, `calibration/`, `audio/`, `drumRunTests/kickOnly/mainDualKickIMU.m`, or any `cfg.Kick.*` / `cfg.IMU.*` value.
- Do NOT change `cfg.EMG.Enabled` (stays `false`; that flag will later gate EMG fusion in the IMU session, not this recorder) or `cfg.EMG.HitThresholdVolts` (leave it, add a one-line comment that it is a placeholder).
- No synthetic EMG in any runtime path. A fake reader is allowed only inside tests.
- Do not commit anything under `data/raw`, `data/metadata`, `data/processed`, `data/models`.
- Never claim a hardware check passed. List hardware checks for the user (section 8).
- Match the surrounding code style: function-per-file, `arguments` blocks, `MyoDrum:*` error identifiers, comment density like `DualIMUSessionLogger.m` and `mainDualKickIMU.m`.
- One logical commit. Existing passing tests must stay passing; report any pre-existing failure separately.

## 4. Where data goes (mirror the IMU recorder exactly)

Read `drumRunTests/kickOnly/mainDualKickIMU.m` (lines 27-49 and the `shutdownDualKick` function) and `common/DualIMUSessionLogger.m` first.

```text
sessionId      = "session_" + datetime('now','Format','yyyyMMdd_HHmmss')
rawFile        = <cfg.Paths.RawData>/<sessionId>/emgAcquisition.mat
metadataFile   = <cfg.Paths.Metadata>/sessions/<sessionId>/sessionMetadata.mat   (variable: metadata)
summary figure = <cfg.Paths.ProcessedData>/<sessionId>/emgOverview.png            (optional, see 6.7)
```

Raw file variables (written incrementally with `matfile`, `-v7.3`, like the IMU logger). **Unfiltered, unsmoothed**; do not add the partner's `movmedian` columns.

| Variable | Size / type | Meaning |
|---|---|---|
| `initialSessionId` | string | same convention as the IMU logger |
| `time_s` | N x 1 double | seconds since session start, midpoint of the four reads |
| `emgVoltage_V` | N x C double | one column per channel, order = `cfg.EMG.ChannelNames` |
| `setDuration_s` | N x 1 double | time taken to read the whole set (first read to last read) |
| `phase` | N x 1 uint8 | 1 = REST, 2 = FREE |
| `marker` | N x 1 uint8 | 0 none, 1 = intended hit, 2 = unintended / false motion, 3 = reference squeeze (maximum comfortable effort) |

`metadata` fields: `sessionId`, `startedAt`, `endedAt`, `mode = "emg-only"`, `config = cfg`, `rawDataFile`, `channelNames`, `analogPins`, `hands`, `phaseNames`, `markerCodes`, `measuredSetRateHz`, `setIntervalStats` (median, p95, p99, max seconds), `restBaseline` (median and robust sigma per channel, or empty if rest skipped), `quality` (per-channel stats + flags, section 6.5), `channelGainNotes` (C x 1 string, from the `GainNotes` option; default `"unrecorded"`), `operatorNotes` (string, from `Notes`), `rateWarning` (string, empty unless the measured set rate is below `cfg.EMG.MinRecommendedSetRateHz`), `loggedSamples`, `completedNormally`, `matlabRelease`, `acquisitionMethod = "MATLAB arduino readVoltage"`, `timestampSource = "host tic/toc; not the IMU device clock"`.

## 5. Files to add or change

| File | Action |
|---|---|
| `common/myoDrumConfig.m` | Fill `cfg.EMG` (section 7). Keep `Enabled=false`. |
| `common/EMGSessionLogger.m` | New. Same shape as `DualIMUSessionLogger`: `append`, `flush`, `close`, `CompletedNormally`, `RawFile`, `MetadataFile`, `LoggedSamples`, preallocated buffer of `cfg.Logging.BufferSamples`, flush when full, save metadata at construction and at close. |
| `EMG_subsystem/connectEMGChannels.m` | New. Validate port (see 6.1), create board, configure pins, return a struct holding the board and a `readSet()` function handle. |
| `EMG_subsystem/readEMGSet.m` | New. Pure-ish: given a reader `fcn(pin)` and pins, returns `voltages (1 x C)` and `[t0 t1]` host times. Reader injected so it is testable. |
| `EMG_subsystem/computeRestBaseline.m` | New, pure. Median and `1.4826*MAD` per column (same method as the gyro bias code in `mainKickIMUFast.m`). |
| `EMG_subsystem/summarizeEMGQuality.m` | New, pure. Per-channel min/median/max/range and flags (6.5). |
| `drumRunTests/emgOnly/mainEMGRecord.m` | New. The session workflow. |
| `drumRunTests/emgOnly/README.md` | New. |
| `mainMyoDrum.m` | Add mode `"emg"` (adds paths, calls `mainEMGRecord`) and a help line. Add `cfg.EMG` pins to `printStatus`. |
| `data/README.md`, `EMG_subsystem/README.md`, `docs/DATASET_SCHEMA.md` | Document the new session layout and variables. |
| `tests/testEMGSubsystem.m` | New, hardware-free (section 8). |

## 6. Behaviour of `mainEMGRecord`

Signature: `mainEMGRecord(options)` using name-value options:

| Option | Default | Purpose |
|---|---|---|
| `Config` | `myoDrumConfig(projectRoot)` | Override, so tests can point paths at a temp folder |
| `Reader` | `[]` | Injected `fcn(pin)->volts` for tests; empty = real Mega |
| `DurationSeconds` | `inf` | Stop automatically (tests, timed runs) |
| `SkipRest` | `false` | Skip the rest baseline phase |
| `ShowPlot` | `true` | `false` = headless, no figure, no key handling |
| `GainNotes` | `strings(C,1)` filled with `"unrecorded"` | Operator's description of each sensor's gain pot (e.g. `"mid"`, `"2 turns from min"`), stored per channel in the metadata |
| `Notes` | `""` | Free-text session note (electrode placement, what was being done) |

1. **Connect.** Check `cfg.Hardware.COMPort` against `serialportlist("available")`. If missing, error with the available ports and the hint that another MATLAB `arduino` object or the IMU session may hold the port (`clear all`). Then create the board and configure the four pins.
2. **Pre-flight (about 2 s).** Read all channels repeatedly and print a table (channel, pin, median V, range V). Warn only; never abort on a bad channel, because the user may be debugging a sensor. **A channel that reads about 0 V while relaxed must be reported neutrally** ("0 V at rest: normal for an envelope; confirm it rises on a squeeze"), never as a fault, because a healthy relaxed envelope sits at zero. Only warn in pre-flight for readings at or above `cfg.EMG.Quality.RailVolts` (gain too high) or a non-finite value. Whether a channel ever responded is judged at the end of the session (6.5).
3. **Rest phase** (unless `SkipRest`): prompt the user to relax, record `cfg.EMG.RestSeconds` seconds with `phase = 1`, compute `restBaseline`, print it. Warn if any channel's rest sigma exceeds `cfg.EMG.Quality.RestSigmaWarnVolts` (do not abort).
4. **Free phase:** record with `phase = 2` until the user presses `Q`, closes the window, or `DurationSeconds` elapses. Key `H` sets `marker = 1` on the next logged set (intended hit), key `X` sets `marker = 2` (unintended / false motion), key `R` sets `marker = 3` (reference squeeze at maximum comfortable effort, used offline to normalise each channel because the gains differ). Print the marker meanings at start.
5. **Quality summary** at the end using `summarizeEMGQuality`, judged over the **whole session**. Flags per channel: `neverResponded` (range across the whole session below `cfg.EMG.Quality.FlatRangeVolts`, i.e. the channel never moved, so unpowered, disconnected or the pot is at minimum; this replaces the old "flat zero at rest" idea, because a healthy relaxed channel is also 0 V), `railed` (max at or above `cfg.EMG.Quality.RailVolts`, gain too high), `noisyRest` (rest sigma above `RestSigmaWarnVolts`). Also report each channel's 95th-percentile and max voltage so the operator can see how much of the 0-5 V range each gain setting uses.
   - **Rate warning:** if the measured set rate is below `cfg.EMG.MinRecommendedSetRateHz`, print a clear warning and fill `metadata.rateWarning`: "Recorded at X Hz. Burst peaks are undersampled lower bounds and timing is coarse; this recording is for amplitude and placement checks, not hit-timing analysis." Do not suppress it.
6. **Timing and speed.**
   - One `tic` at session start. For each set: `t0 = toc` before the first read, `t1 = toc` after the last; `time_s = (t0+t1)/2`, `setDuration_s = t1-t0`. Always read channels in the same fixed order.
   - No array growth with `end+1`. Use the preallocated logger buffer.
   - The live plot must not stall acquisition (the IMU work found plotting caused 0.3-0.4 s stalls). Update at most every `cfg.EMG.PlotEverySets` sets, rolling window `cfg.EMG.LiveWindowSeconds`, fixed y-limits `[0 5]`, left-arm and right-arm axes, units V, and use `drawnow limitrate`. Plotting must happen after the sample has been logged.
7. **Shutdown (always).** Use `onCleanup` like `shutdownDualKick`: flush and close the logger, `clear` the board, delete the figure, and print: samples logged, measured set rate (median interval based), interval median/p95/p99/max, per-channel min/median/max, quality warnings, and the raw and metadata paths. Writing the optional `emgOverview.png` (one axes per hand, raw voltages, markers drawn as vertical lines) goes inside try/catch and must never prevent the data from being saved. Set `completedNormally = true` only on a clean stop.

## 7. Config additions (`cfg.EMG`)

```matlab
cfg.EMG.Enabled = false;                 % unchanged: gates IMU+EMG fusion later
cfg.EMG.AnalogPins = ["A0"; "A2"; "A3"; "A4"];   % left forearm moved A1 -> A0
cfg.EMG.ChannelNames = ["LEFT_FOREARM"; "LEFT_TRICEPS"; "RIGHT_FOREARM"; "RIGHT_TRICEPS"];
cfg.EMG.Hands = ["LEFT"; "LEFT"; "RIGHT"; "RIGHT"];
cfg.EMG.RestSeconds = 5;
cfg.EMG.LiveWindowSeconds = 15;
cfg.EMG.PlotEverySets = 3;
cfg.EMG.MinRecommendedSetRateHz = 50;    % below this, peaks are undersampled (warn)
cfg.EMG.Quality.FlatRangeVolts = 0.005;
cfg.EMG.Quality.RailVolts = 4.8;
cfg.EMG.Quality.RestSigmaWarnVolts = 0.15;
```

Keep the vectors as columns (matching the existing `strings(0,1)`). Add a short comment that voltages are analog envelope outputs.

## 8. Tests (hardware-free, `tests/testEMGSubsystem.m`, `functiontests`)

1. Config: four unique pins, name/hand vectors match pin count, `Enabled` is still `false`.
2. `EMGSessionLogger` schema across a buffer boundary: sizes N x C, `phase` and `marker` are `uint8`, metadata has `loggedSamples`, `completedNormally`.
3. `computeRestBaseline`: known values, and robust to one outlier sample.
4. `summarizeEMGQuality`: a never-moving channel gets `neverResponded`; a railed channel gets `railed`; **a channel that is 0 V while relaxed but rises to about 2 V in a squeeze is healthy (no flags)**; a noisy-rest channel gets `noisyRest`.
5. `mainEMGRecord` with an injected fake reader, `ShowPlot=false`, a short `DurationSeconds`, and `Config` paths pointed at a temp folder: files appear at the section 4 locations, `emgVoltage_V` has the right columns, `time_s` is monotonic, nothing is written under the real `data/`.
6. Marker plumbing: scripted markers 1, 2 and 3 land on the intended rows.
7. `GainNotes` and `Notes` are stored in the metadata; when omitted the gain notes are `"unrecorded"`.
8. Rate warning: a fake reader that sleeps long enough to fall below `MinRecommendedSetRateHz` produces a non-empty `rateWarning`; a fast one produces an empty string.
9. The config pin map is exactly `["A0";"A2";"A3";"A4"]` with `A1` and `A5` unused.

Acceptance: `tests/runAllTests.m` passes with the new tests and no previously-passing test broken.

## 9. Hardware validation (the user runs; do not claim)

| ID | Procedure | Pass |
|---|---|---|
| E1 | Close any other `arduino` session. Run `mainMyoDrum("emg")`, stay relaxed 5 s, then free-record 60 s: for each muscle one or two firm squeezes marked with `R`, a few normal strokes marked `H`, and a few deliberate false motions marked `X`; then `Q`. | Files at `data/raw/session_*/emgAcquisition.mat` and `data/metadata/sessions/session_*/sessionMetadata.mat`; a measured set rate and the rate warning (if any) are printed |
| E2 | Load the file. | `emgVoltage_V` is N x 4; each squeeze visibly raises only its own channel; markers present; pin A0 data is the left forearm |
| E3 | Check the quality table against the sensors. | A dead or railed channel is reported, not hidden; healthy zero-at-rest channels are not flagged |
| E4 | Note each channel's gain pot position in `GainNotes` for the run. | `channelGainNotes` appears in the metadata |

## 10. Out of scope

IMU+EMG fusion, EMG gating of hits, firmware or serial streaming (planned separately as part of `docs/CODEX_FIX_PLAN.md` WP-E), filtering or normalisation of the recorded signal, and classification.
