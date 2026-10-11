# Codex implementation plan v2: MyoDrum acquisition rate and risk fixes

Revision: v3, 2026-10-10. v2 was written after evaluating four recorded sessions and the WP-0 work; v3 adds section 5A (orientation drift and drum-region tracking, WP-O0 to WP-O6) after the first three-drum sessions showed drums "drifting". **Section 5A is the highest priority and runs before WP-E onward** because it works at the current 13-15 Hz rate and does not need the firmware.
**Implemented directly by Claude on 2026-10-10 (do not redo):** `common/GravityAzimuthEstimator.m` (replaces `imufilter` in `mainDualKickIMU.m`), the hand + gravity-posture drum selection (`calibration/buildPostureModel.m`, `classifiers/ruleBased/classifyPosture.m`, new `cfg.Regions.Hand*/Posture*` and `cfg.Orientation.Estimator*` settings, posture calibration quality gates and stale-model rejection), configurable gyro range (`setupMPU6050Pair`, `decodeMPU6050Frame`, default still 250 dps), offline tools `tools/replayOrientation.m` and `tools/orientationReplayReport.m`, `tests/testOrientationAndPosture.m`, and the replay test now skips the two sessions recorded with superseded detector code. Azimuth-based regions (WP-O2 tracking) were deliberately NOT built on top: the replay showed azimuth still wanders at 13-15 Hz with a clipped gyro, so drum choice no longer depends on it. Re-evaluate azimuth only after firmware streaming (WP-E/F) and a wider gyro range.
Audience: Codex (implementation). Claude assessed only; do not assume any WP below is implemented unless listed above. Some of WP-A to WP-D and the old WP-2/WP-4 appear partly done in the working tree (`resolveGyroBias.m`, `decodeMPU6050Frame.m`, `OrientationWindowAverager.m`, live read mode, three-drum mode); verify each against its acceptance criteria and the test suite before building on it.

## 1. Evidence this plan is built on

Measured from `data/raw/session_*/dualKickIMU.mat` (latest session `session_20261010_135959`):

| Finding | Value |
|---|---|
| Effective acquisition rate | about 13.6 Hz (median dt 74 ms; startup measurement 10.4-10.7 Hz) |
| Samples across a 120 ms stroke | about 1.6 |
| Gyro at +/-250 deg/s full scale (4.36 rad/s) | 13-16% of samples; 77-88% of hit windows |
| Accel at the +/-2 g limit | 22-24% of samples |
| Loop stalls | p99 125 ms (was 365 ms before plot throttling); 5 stalls above 250 ms in 268 s, max 565 ms |
| Cycles where both hands hit | 8% of hit cycles |
| Orientation filter rate | about 3.4 Hz (13.6 Hz / stride 4) |

Root cause: every sample is a serial round trip through the MATLAB Arduino support package (`readRegister`). Throughput is capped by transport overhead, not by the MPU6050 and not by detector tuning. All downstream work (target classification, EMG fusion, orientation features) is undersampled until acquisition changes.

## 2. Status of the previous plan

| Old WP | State | Disposition in v2 |
|---|---|---|
| WP-0 replay harness, `runAllTests` | Done, uncommitted | Keep. Fix the failing test (WP-A) |
| WP-1 gyro bias policy | Not done | WP-B |
| WP-2 live read path | Not done | Superseded by WP-E/WP-F; keep legacy path only as fallback |
| WP-3 arm timeout | Not done | WP-C |
| WP-4 orientation feed | Not done | Redone as WP-H on the new stream |
| WP-5 trial schema | Not done | WP-I |
| WP-6 baseline equivalence | Not done | WP-J |
| WP-7 more tests | Not done | WP-K |
| WP-8/9 docs, local config | Not done | WP-L |
| Unplanned, already done by the user | Plot throttling (`PlotEverySamples = 1000`, `ControlPollEverySamples = 10`), buffer 5000, one audio voice per both-hand cycle, `mainIMU` redirect, pre-loop `drawnow` | Keep; see audio policy in WP-G |

Current test result: 11 pass, 1 fail (`testRecordedSessionReplay`). The two older sessions (`130411`, `131918`) were recorded with earlier detector code (`MinInterHitSeconds` 0.08, then a `FastRearmSeconds` guard that no longer exists). Sessions `134507` and `135959` replay exactly.

## 3. Ground rules

- MATLAB R2024b on Windows; Arduino Mega 2560 on `COM7` (configurable). Run tests with `tests/runAllTests.m` (add `tests` to the path first).
- Do NOT change `cfg.Kick.*` threshold values except inside WP-G, and only as a documented, data-backed proposal that the user approves. Otherwise new parameters must preserve current behaviour.
- Do NOT edit `drumRunTests/kickOnly/mainKickIMUFast.m` (frozen behavioural baseline). `testWorkingKickBaselineIsPreserved` must keep passing.
- Keep working modes `kick-only`, `dual-kick`, `imu-diagnostics`, `status`. New acquisition ships as a NEW mode first; the legacy path stays the default until the user validates the new one.
- Do not commit anything under `data/raw`, `data/metadata`, `data/processed`, `data/models`.
- No synthetic EMG. Snare, crash, ML and EMG work are out of scope until the WP-G gate passes.
- Everything testable offline must have a hardware-free test. Anything needing the Mega is listed under "Hardware validation" and left to the user. Never claim a hardware check passed.
- Read the code before relying on this plan. Verify field names, signatures and behaviour yourself.
- One WP per commit, tests green between commits.

## 4. Decisions (defaults; proceed with these unless the user overrides)

| # | Decision | Default |
|---|---|---|
| D1 | Gyro bias policy | `"fresh"` every session; `"cached"` is opt-in with `GyroBiasMaxAgeHours = 8` |
| D2 | `ArmTimeoutSeconds` | 1.0 s at the legacy 13 Hz rate; must be re-validated at the new rate |
| D3 | Acquisition architecture | Arduino firmware streams timestamped binary frames; MATLAB reads with `serialport` |
| D4 | Stream target | 200 Hz per IMU, both IMUs in each frame, 400 kHz I2C |
| D5 | Sensor ranges | Configurable; proposed new default +/-500 deg/s and +/-4 g, set only after hardware validation |
| D6 | Baseline | `mainKickIMUFast.m` stays frozen; prove equivalence by test |
| D7 | Audio on both-hand cycles | New cfg flag `cfg.Kick.MergeSimultaneousVoices`, default `true` (current behaviour); both hit flags always logged |

## 5. Work packages

Order (v3): verify WP-A to WP-D status, then WP-O0 to WP-O4 (section 5A), user gate H4-H5, then WP-O6, then WP-E, WP-F, (user gate H1-H3), WP-G, WP-I, WP-J, WP-K, WP-L. WP-H is superseded by WP-O1 (the new estimator is timestamp-driven and rate-agnostic, so it works unchanged on the 200 Hz stream).

### WP-A: Commit baseline and make the replay test version-aware

Problem: the replay test fails on sessions recorded by superseded detector code, so a red test no longer signals a regression.

Files: `common/myoDrumConfig.m`, `drumRunTests/kickOnly/mainDualKickIMU.m` (metadata), `classifiers/ruleBased/updateKickDetector.m` (version constant only), `tests/testRecordedSessionReplay.m`, `tests/helpers/replayDetector.m`.

Changes:
1. Add `classifiers/ruleBased/kickDetectorVersion.m` returning a string (start at `"2"`; bump it whenever detector behaviour changes).
2. Write `metadata.detectorVersion` into the session metadata at session start.
3. The replay test replays only sessions whose `detectorVersion` equals the current one. Sessions without it are reported through `assumeTrue`/log text as "recorded with an older detector; skipped", never as failures. Print which sessions were replayed and which were skipped.
4. Do not delete or alter existing recordings.

Acceptance: suite is 12 pass / 0 fail on this machine. Sessions `134507` and `135959` are not skipped only if they carry a version; since they lack one, add a one-time documented mapping file `tests/fixtures/sessionDetectorVersions.json` marking those two as version "2" and the two older ones as "1".

### WP-B: Gyro bias policy (was WP-1)

Files: `common/myoDrumConfig.m`, `calibration/calibrateDualIMUSession.m`, new `calibration/resolveGyroBias.m`, `calibration/README.md`, `README.md`, `docs/CALIBRATION_PROTOCOL.md`, header of `mainDualKickIMU.m`.

Changes:
1. `cfg.Calibration.GyroBiasPolicy = "fresh"`, `cfg.Calibration.GyroBiasMaxAgeHours = 8`.
2. Pure function `resolveGyroBias(measureFcn, cacheFile, policy, maxAgeHours, forceMeasure)` returning bias, noise, source (`"measured"`/`"cached"`) and age.
3. `fresh` always measures and refreshes the cache file. `cached` loads only if shape, finiteness and `createdAt` age are valid. `ForceStatic = true` forces a measurement under either policy.
4. Record policy, source and age in `imuSessionCalibration.mat` and session metadata.
5. Correct every doc or header that describes the behaviour.

Acceptance: unit tests (fake `measureFcn`) cover fresh-ignores-cache, cached-uses-young-cache, cached-rejects-old/NaN/wrong-size, force override. Existing cache files still load.

### WP-C: Detector arm timeout (was WP-3)

Files: `common/myoDrumConfig.m`, `classifiers/ruleBased/updateKickDetector.m`, `kickDetectorVersion.m`, new `tests/testKickDetector.m`.

Changes:
1. `cfg.Kick.ArmTimeoutSeconds` (D2). In mode 2, if `timeNow - detector.armedAtTime > ArmTimeoutSeconds`, return to mode 1 with no hit.
2. If the field is absent from a supplied config (old saved metadata), behave exactly as before (no timeout).
3. Bump `kickDetectorVersion` only if replay of version-2 sessions changes; otherwise state in the PR that replay is bit-identical.

Acceptance: the three existing detector tests pass unmodified; new tests for hover-then-timeout and in-time genuine stroke; replay of sessions `134507`/`135959` produces identical hits, or the PR reports the exact difference.

### WP-D: Configuration-driven sensor ranges (new)

Problem: `cfg.IMU.GyroscopeRangeDps` and `cfg.IMU.AccelerometerRangeG` are unused. `setupMPU6050Pair` hard-codes registers `0x1B`=0 and `0x1C`=0, and `readMPU6050Sample` hard-codes `/131` and `/16384`. Strokes clip at full scale.

Files: `common/myoDrumConfig.m`, `IMU_subsystem/setupMPU6050Pair.m`, `IMU_subsystem/readMPU6050Sample.m`, new `IMU_subsystem/mpu6050Scale.m`, calibration loaders, `data` README.

Changes:
1. `mpu6050Scale(gyroRangeDps, accelRangeG)` returns register codes and LSB scale factors for the legal MPU6050 ranges (gyro 250/500/1000/2000, accel 2/4/8/16) and errors on anything else.
2. `setupMPU6050Pair` takes the ranges, writes the matching registers, and stores the scale factors in each `sensors(i)` struct; `readMPU6050Sample` uses them. The default config values remain 250 and 2 so current behaviour is unchanged until the user changes them.
3. Store the range used inside every saved six-pose and gyro-bias calibration. The loaders must reject (and re-run) a cache whose range differs from the configuration, with a clear message.
4. Log the ranges in session metadata.
5. Add saturation reporting: count samples at or beyond 98% of full scale per axis and print at session end.

Acceptance: tests for `mpu6050Scale` (all legal values, an illegal value), for decode scaling, and for cache rejection on range mismatch. Existing 250/2 behaviour is numerically identical.

### WP-E: Firmware streaming acquisition and stream reader (new, central)

Goal: raise the effective acquisition rate from about 13 Hz to at least 100 Hz (target 200 Hz) with device-side timestamps.

New files:
- `firmware/MyoDrumIMUStream/MyoDrumIMUStream.ino`: uses only `Wire`. Configures both MPU6050s (address 0x68 and 0x69; ranges and DLPF supplied at build time or via a start command), runs I2C at 400 kHz, reads both 14-byte bursts on a fixed `micros()` schedule, and emits one binary frame per cycle.
- `docs/FIRMWARE_PROTOCOL.md`: the frame format, serial baud, command set, and error behaviour.
- `IMU_subsystem/decodeIMUStreamFrame.m` (pure), `IMU_subsystem/IMUStreamReader.m` (class wrapping `serialport`), `IMU_subsystem/crc8Imu.m` or the chosen checksum.
- `tests/testIMUStreamDecoder.m`, `tests/helpers/makeSyntheticStream.m`.

Protocol requirements:
- Frame: sync bytes, protocol version, 16-bit sequence counter, 32-bit `micros()` timestamp, 2 x (accel XYZ, temperature, gyro XYZ) as raw int16, status byte, checksum. Fixed length, little-endian, documented byte by byte.
- Baud: 250000 (exact on a 16 MHz AVR) or 115200; 200 Hz x 2 IMUs is about 7-8 kB/s, so 115200 is sufficient. Choose one, make it a config value, and record the measured loss at that baud.
- Commands from MATLAB: start, stop, ping/identify (returns protocol version and firmware build). The sketch must emit nothing until started.
- Per-IMU status flags for an I2C failure; on failure send the frame with the flag set rather than stalling.

MATLAB reader requirements:
- Resynchronise after garbage or a partial frame; count checksum failures, sequence gaps (dropped frames), and parse resets.
- Expose non-blocking `readAvailableFrames()` returning all complete frames with device timestamps, plus `getStatistics()` (frames received, dropped, checksum failures, measured rate, max inter-frame gap).
- Must not coexist with an open `arduino()` object on the same port; the new session path must not create one.

Acceptance (offline): decoder tests for a good frame, bad checksum, truncated frame, sequence wrap at 65535, garbage prefix, back-to-back frames, split reads; signed int16 boundary values (0x8000, 0x7FFF); range scaling consistent with WP-D.

### WP-F: Streaming dual-kick session (new)

Goal: run the existing calibration, detector, audio and logging pipeline from the stream, without disturbing the legacy mode.

Files: `mainMyoDrum.m` (new mode `"dual-kick-stream"`), new `drumRunTests/kickOnly/mainDualKickStream.m`, `common/myoDrumConfig.m` (`cfg.IMU.Acquisition`, `cfg.IMU.StreamBaud`, `cfg.IMU.StreamRateHz`), `common/DualIMUSessionLogger.m`.

Changes:
1. Adapter that presents the stream as `readSample`-style closures (`sensor(i)` -> next frame's sample for IMU i) so `calibrateDualIMUSession`, six-pose loading and direction learning are reused unchanged.
2. Main loop processes all frames available each iteration, in order. Detector time comes from the frame's device timestamp, not `toc`. `sampleRateHz` is the measured stream rate, with the clamping logic in `updateKickDetector` unaffected.
3. Audio is triggered immediately when a frame produces a hit, then logging and plotting happen after (same ordering principle as the current loop).
4. Logger additions: device timestamp, sequence number, `frameValid` per IMU, per-iteration loop duration, dropped-frame count. Existing fields keep their names and shapes.
5. Time-based flush: flush the buffer at most once every N seconds (default 30), only on an iteration with no hit in the last 0.5 s, so a crash loses at most that window.
6. Session end report: measured rate, drops, checksum failures, loop duration p50/p95/p99/max, saturation counts (WP-D), hits per hand, same-cycle doubles.

Acceptance (offline): a loop-level test driven by a fake `IMUStreamReader` replays a synthetic stroke sequence and asserts one hit per stroke, correct device-timestamp use, and logger schema. Legacy `dual-kick` and `kick-only` are untouched and their tests pass.

### Hardware validation gate (user runs; Codex does not claim)

| ID | After | Procedure | Pass criterion |
|---|---|---|---|
| H1 | WP-E/F | Flash the sketch; run `mainMyoDrum("dual-kick-stream")` idle for 2 minutes | Measured rate at or above 100 Hz (target 190+); dropped frames below 0.5%; zero checksum failures after sync |
| H2 | WP-F | Play normally for 3 minutes | Loop p99 well below 50 ms; no audible latency regression versus the legacy mode |
| H3 | WP-D | Play hard strokes with the new ranges | Gyro and accel saturation counts below 1% of samples |

Do not start WP-G until H1-H3 results are reported.

### WP-G: Detector re-validation at the new rate (new)

Problem: thresholds and guard times were tuned for 13 Hz. At 100-200 Hz the same values may mis-trigger or miss, and the signal now shows real strike structure.

Files: new `docs/KICK_TUNING_PROTOCOL.md`, new `tools/analyzeKickRecording.m`, `common/myoDrumConfig.m` (flag only), `classifiers/ruleBased/updateKickDetector.m` if and only if the user approves a change.

Changes:
1. Write a short controlled-recording protocol: for each hand, N metronome-paced strokes at slow, medium and fast tempos, with the player noting N, so detected count versus expected count is measurable.
2. `analyzeKickRecording(sessionFolder, expectedPerHand)` reports detected vs expected, inter-hit minimum/median, gaps under the guard time (double triggers), and latency from gyro reversal to hit.
3. Add `cfg.Kick.MergeSimultaneousVoices` (D7, default true). When false, play one voice per hand hit; always log both hit flags.
4. Produce a threshold sweep report (arm, reversal, deceleration, jerk, return travel, guard time) on the recorded sessions, listing the effect on counts. Do NOT apply any change. Present a recommended set and the evidence; the user approves.
5. Any approved threshold change bumps `kickDetectorVersion`.

Acceptance: analysis tool tested on synthetic data with a known stroke count; report generated for the user's recordings.

### WP-H: Orientation on the full-rate stream (was WP-4)

Files: `drumRunTests/kickOnly/mainDualKickStream.m`, `common/myoDrumConfig.m`.

Changes: feed `imufilter` all frames from each loop iteration in one batched call per IMU (it accepts N x 3 inputs), with `SampleRate` equal to the measured stream rate. Remove the stride-4 decimation and the matching warm-up skipping in the stream path. Keep orientation strictly after audio triggering. Keep the legacy path unchanged.

Acceptance: offline test, constant synthetic rotation rate for T seconds gives the expected angle within a stated tolerance; the old decimated-instantaneous feed fails the same test (documents why).

### WP-I: Trial-based dataset schema (was WP-5)

Files: `docs/DATASET_SCHEMA.md`, `common/DualIMUSessionLogger.m`, new `common/TrialLog.m`, new `common/validateSessionSchema.m`, `common/myoDrumConfig.m` (`cfg.Dataset.SchemaVersion`).

Changes: versioned schema; trial table (`trialId`, `sessionId`, `round`, `target`, `hand`, `trialType` calibration/training/validation/heldOut, `startTime_s`, `endTime_s`, `eventTime_s`, `valid`, `exclusionReason`, `notes`) written beside the continuous stream; session metadata includes software revision (git hash if obtainable, else "unknown"), detector version, acquisition mode, stream statistics, ranges, calibration references and ages, gyro bias policy and source, hand-to-address map. `validateSessionSchema` checks a session folder. One trial per detected hit for the kick-only session. Legacy sessions are reported as "legacy, no trial table", never a crash.

Acceptance: logger schema test extended; validator passes a fresh session and fails a deliberately broken one clearly.

### WP-J: Detector ownership and equivalence (was WP-6)

Files: new `tests/helpers/baselineKickReference.m`, new `tests/testBaselineEquivalence.m`, kickOnly and ruleBased READMEs.

Changes: hardware-free reference transcription of the `mainKickIMUFast.m` state machine (cite source lines), compared with `updateKickDetector` on synthetic sequences and on version-matching recordings. Document that `classifiers/ruleBased/updateKickDetector.m` is canonical and the baseline is frozen. Report divergences with the first differing sample; do not silently fix.

### WP-K: Broader test coverage (was WP-7)

Add: accel-calibration loader (valid, invalid, range mismatch, pose rejection via a fake `readSample`), direction learning (extract the axis/sign estimation to a pure function; each axis and sign, too-small stroke, corrupt cache), `applyMPU6050AccelCalibration` edge cases, legacy decode scaling. Tests needing Sensor Fusion Toolbox use `assumeTrue`. Suite must run without hardware.

### WP-L: Documentation, legacy, local configuration (was WP-8/9)

- `PROJECT_STRUCTURE.md`: remove non-existent `calibration/legacy/`, add `drumRunTests/diagnostics/`, `firmware/`, `tools/`, and the `data/metadata/{calibration,sessions,legacy}` layout.
- Grep callers before moving anything; prefer deprecation headers. `IMU_subsystem/sixPoseCalibration.m`: deprecate or add the plausibility checks from `loadOrRunMPU6050AccelCalibration.m`.
- Optional git-ignored `common/myoDrumLocalConfig.m` override (COM port, board, baud), with a tracked `.example`; add the real file to `.gitignore`.
- `docs/HARDWARE_SETUP.md`: wiring (SDA/SCL, AD0 high for 0x69), required toolboxes (MATLAB Support Package for Arduino for the legacy path, Sensor Fusion and Tracking Toolbox, `serialport` support), flashing steps, baud, first-run calibration, moving calibration between machines.
- `docs/REPOSITORY_AUDIT.md`: dated resolved / still-open section.

## 5A. Orientation drift and drum-region tracking (v3, highest priority)

### Evidence (from the recorded three-drum sessions, 2026-10-10)

| Finding | Value |
|---|---|
| Session `145313`, yaw-based regions, 191 s | 57/134 LEFT hits and 34/126 RIGHT hits (43% / 27%) rejected as ambiguous or outside every region; region spreads in yaw +/-56 to +/-109 deg; LEFT SNARE and CRASH centres 11 deg apart |
| Session `150626`, pitch/roll regions, 57 s | calibration self-accuracy 43% (LEFT), 53% (RIGHT) vs 33% chance; LEFT produced only CRASH labels and RIGHT only KICK labels; KICK/CRASH centres on LEFT within 8 deg pitch and 6 deg roll; the model was still saved and used |
| Orientation while the hands were still (gyro below about 1-3 deg/s, accelerometer constant) | angles kept slewing 1-2.5 deg/s mid-session; right after warm-up the first update jumped by 59 deg pitch relative to the captured neutral, then yaw slewed about 20 deg/s |
| Leftover gyro bias while quiet | at most about 0.2 deg/s in every session, so gyro bias is NOT the cause |
| Orientation filter | `imufilter` fed 4-sample window means at about 3.7 Hz; fixed `SampleRate` taken from the startup rate measurement |
| Startup rate vs live loop rate | session `145313`: 10.8 Hz vs 14.9 Hz (filter dt 38% too long, angles about 1.4x too large); `150626`: 14.7 vs 15.1 Hz (fine) |
| Sensor attitude | gravity lies mostly along sensor Z (0.8-0.96 g), no Euler gimbal-lock problem |

Interpretation (partly inferred; WP-O0 exists to confirm it offline):
1. The filter is not converged when neutral is captured, and at 3.7 Hz it re-converges slowly after every stroke, so all angles slew relative to a wrong zero.
2. Yaw has no reference on an MPU6050, so left/front/right (azimuth) can only come from integrated gyro and needs drift tracking.
3. Moving the classifier to pitch/roll removed the drift but also removed the information: rotating about the vertical axis does not change gravity, so pitch/roll cannot separate drums laid out left/front/right.
4. A wrong filter `SampleRate` scales every angle.

### Proposed acceptance targets (adjust with the user)

| Metric | Baseline today | Target |
|---|---|---|
| Tilt (pitch/roll) slew during quiet periods | 1-2.5 deg/s | below 0.3 deg/s |
| Azimuth (yaw) drift during quiet periods | 2.4 deg/s mid-session, about 20 deg/s after warm-up | below 0.5 deg/s, after tracking below 5 deg/min effective |
| Neutral capture error (angle at capture vs 5 s later, held still) | up to 59 deg | below 2 deg |
| Guided three-drum accuracy, 3 minutes, per hand | not measured; self-accuracy 43-53% | at least 85% correct, at most 10% rejected, last minute within 10 points of first minute |

### WP-O0: Offline orientation replay and metrics (do first)

Goal: reproduce the drift offline from logged data and make every later change measurable without hardware.

Files: new `tools/orientationReplayReport.m`, new `tests/helpers/quietRuns.m`, new `tests/testOrientationMetrics.m`.

Changes:
1. Input a session folder. Using `accelCalibrated_g`, `gyroCorrected_rad_s` and `time_s`, run any estimator (a function handle with the WP-O1 interface) over the whole session with true per-sample dt, and also re-run the legacy path (`imufilter` on stride-4 window means at the recorded startup rate) for comparison.
2. Report per hand: tilt and azimuth slew during quiet runs (gyro norm below 3 deg/s for at least 1.5 s, skipping the first 5 samples of each run), neutral-capture error (orientation at end of a simulated 2 s and 10 s warm-up vs 5 s later), and the change of pitch/roll/azimuth across each detected hit and 2 s later.
3. Print a table of the legacy numbers so the baseline in the table above is reproduced or corrected. If the numbers differ materially from the table, update this document in the same commit.
4. Synthetic tests: a still sensor gives zero slew; a constant known rotation rate gives the right angle; accelerometer spikes do not move tilt when gated.

Acceptance: the report runs on sessions `135959`, `145313`, `150626` (skipped if absent) and reproduces slewing in the legacy path.

### WP-O1: Timestamp-driven gravity/azimuth estimator

Goal: replace the decimated `imufilter` for the three-drum path with a per-sample estimator that stays converged and gives a usable azimuth.

Files: new `common/GravityAzimuthEstimator.m` (handle class), `common/myoDrumConfig.m` (`cfg.Orientation.Estimator`, `cfg.Orientation.GravityGain`, `cfg.Orientation.AccelGateG`, `cfg.Orientation.GyroGateRadPerSec`, `cfg.Orientation.ConvergenceDeg`, `cfg.Orientation.MaxWarmupSeconds`), `drumRunTests/kickOnly/mainDualKickIMU.m`, tests.

Design:
1. State: body-frame gravity direction (unit vector `u`, pointing up), azimuth angle about `u`, last timestamp.
2. Per sample (every acquisition, no stride decimation, the filter cost is a few flops):
   - dt from the sample timestamp (`timeNow` on the legacy path, device timestamp on the stream path), clamped to a documented maximum; never from the startup-measured rate.
   - Propagate `u` with the gyro (`u <- u - omega x u * dt`, renormalise).
   - Correct `u` toward the measured accelerometer direction with a small gain, gated: weight is zero when `abs(norm(a) - 1 g)` exceeds `AccelGateG` or the gyro magnitude exceeds `GyroGateRadPerSec`, so strokes and impacts do not pollute tilt.
   - Azimuth increment = `dot(omega, u) * dt` (yaw rate about the vertical), integrated and wrapped.
3. Initialise `u` from the first accelerometer samples (not identity), azimuth 0.
4. Outputs: tilt as pitch and roll (documented convention, display and logging) and the gravity vector, azimuth relative to a settable neutral, plus a `Converged` flag (tilt change below `ConvergenceDeg` for 1 s).
5. `cfg.Orientation.Estimator` = `"gravityAzimuth"` or `"imufilter"`. Keep `imufilter` selectable. Do not make the new estimator the default for `three-drum` until WP-O0 shows it beats the legacy numbers, and say so in the PR.
6. Warm-up: run until `Converged`, with a maximum, then capture neutral. If the maximum elapses unconverged, warn and state the residual error rather than silently proceeding.
7. The live loop and region calibration use the same estimator class so calibration and play share one definition of the features.

Acceptance: WP-O0 report shows the quiet-period slew and neutral-capture targets met on the recorded sessions, or the PR states exactly which were not and why. Unit tests: convergence from a tilted start, gating on accelerometer spikes, constant-rate azimuth, dt-clamp behaviour, identical result for equal timestamps on either source.

### WP-O2: Drum regions with drift-tracked azimuth

Goal: restore left/front/right discrimination and keep it stable while azimuth drifts.

Files: new `classifiers/ruleBased/DrumRegionTracker.m`, update `calibration/buildDrumRegionModel.m`, `classifiers/ruleBased/classifyDrumRegion.m`, `common/myoDrumConfig.m` (`cfg.Regions.*`), tests.

Design:
1. Region features per hand: azimuth (circular), pitch and roll. The model stores per-drum centres and robust spreads, and the estimator name and version used to build it.
2. Feature timing: evaluate `cfg.Regions.FeatureTime` = `"arm"` (orientation when the downstroke armed, from a short orientation history ring buffer) vs `"hit"` using the calibration data in WP-O0, and default to whichever separates the drums better; the stroke itself disturbs orientation at hit time.
3. Tracking: each hand keeps a scalar `azimuthOffset`. Classify with `measured - offset`. After a confident classification (normalised distance below `cfg.Regions.TrackMaxDistance`, margin above `cfg.Regions.TrackMinMargin`), update `offset += alpha * wrap(measured - offset - centre(label))`, with `alpha` small and a per-update step limit so one wrong hit cannot drag the map. Expose reset (the R key) and a read-only `Offset` for logging.
4. Rejected or ambiguous hits are logged with the distances and never update the offset.
5. Tests with synthetic data: drift of 0.5 deg/s for 120 s with hits on three drums; the static classifier degrades below 70% while the tracked one stays above 90%; a burst of misclassified hits does not run away; reset restores the original map.

Acceptance: tests above pass; no change to the kick detector.

### WP-O3: Region calibration quality gates and cache validity

Files: `drumRunTests/kickOnly/mainDualKickIMU.m` (`collectDrumRegionCalibration`), `calibration/buildDrumRegionModel.m`, `common/myoDrumConfig.m`.

Changes:
1. After building the model compute, per hand, self-accuracy and the minimum pairwise separation of drum centres in normalised units and in degrees of azimuth.
2. If self-accuracy is below `cfg.Regions.MinTrainingAccuracy` (default 0.75) or separation is below `cfg.Regions.MinSeparationDeg`, do not save the persistent model or run play mode with it. Print which pair overlaps and ask for repeat calibration (loop with a retry limit), with a documented override flag.
3. Store the estimator name/version, `FeatureTime`, neutral convention and `createdAt` in the model. The loader must reject a model whose estimator or feature definition differs from the current configuration and force recalibration with a clear message.
4. Calibration hit collection must discard hits made before the estimator reports `Converged`.

Acceptance: tests for each gate with synthetic overlapping and well-separated data, and for the stale-model rejection.

### WP-O4: Logging and metadata for drift analysis

Files: `common/DualIMUSessionLogger.m`, `docs/DATASET_SCHEMA.md`, `mainDualKickIMU.m`.

Changes: log per sample azimuth, pitch, roll and a `converged` flag; log per hit the label, confidence, distances to every drum, feature time used and the tracked `azimuthOffset`; log recenter events; session metadata records estimator name/version, warm-up duration and residual convergence, and the region model's self-accuracy. Existing field names and shapes are unchanged. Extend the schema document and the logger schema test.

### WP-O5: Magnetometer option (documentation only)

New `docs/MAGNETOMETER_OPTION.md`: when software tracking is not enough, compare a 9-axis part (MPU-9250, ICM-20948) or an external magnetometer beside each MPU6050 on the same I2C bus; calibration (hard and soft iron), disturbance from drum hardware and laptop, wiring and address constraints, effect on the 13-15 Hz loop. No code until the user decides.

### WP-O6: Guided three-drum validation mode

Goal: measure real classification accuracy and its change over time, which nothing in the repo does today.

Files: new `mainMyoDrum("three-drum-guided")` mode, new `drumRunTests/kickOnly/mainThreeDrumGuided.m`, new `tools/analyzeGuidedSession.m`, trial schema pieces from WP-I as needed.

Changes: after the normal three-drum calibration, prompt a randomised target sequence on the console and a small window (for example "LEFT hand: SNARE"), log intended label, predicted label, confidence, rejection, hit timestamp and hand per trial, then print and save a confusion matrix per hand, rejection rate, and accuracy in each minute of the session (the drift test). The sound still plays so it feels like normal playing.

Acceptance: analyzer unit-tested on synthetic logs with known accuracy.

### Hardware validation for 5A (user runs)

| ID | After | Procedure | Pass criterion |
|---|---|---|---|
| H4 | WP-O1 | Warm up, then hold both sticks still for 60 s | tilt change below 2 deg and azimuth change below 5 deg over the 60 s; neutral reported converged |
| H5 | WP-O2/O3 | Calibrate, then 3 minutes of `three-drum` play with deliberate hits at all three drums | calibration passes the quality gates; rejected hits at most 10%; no systematic walk of the labels toward one drum |
| H6 | WP-O6 | Run guided mode for 3 minutes | accuracy and rejection meet the table above in every minute |

## 6. Exit criteria for this plan

1. `tests/runAllTests.m` passes with no failures on a machine with no Arduino.
2. H1-H3 reported by the user with numbers.
3. WP-G report shows detected-vs-expected counts and double-trigger rate at the new rate.
4. Only then: begin the three-target (kick, snare, crash) calibration and dataset work on the WP-I schema.

## 7. Out of scope here

Snare/crash classifiers, ML training, EMG, any claim of accuracy, and any change to tuned kick thresholds without data and user approval.
