# Preliminary Feasibility Report: MyoDrum IMU and EMG System

## 1. Objective

This investigation assessed whether two MPU6050 inertial measurement units (IMUs) and four surface electromyography (EMG) channels could form a responsive, two-handed virtual drum interface. The work focused on three sounds: kick, snare and crash. The principal questions were whether the system could detect rapid and simultaneous stick movements, distinguish drum regions consistently, reject unintended movement and operate with sufficiently low delay for live performance.

## 2. Implemented improvements

The original MATLAB/Arduino Support Package implementation read the two IMUs sequentially. It was progressively extended with persistent six-pose accelerometer calibration, fresh gyroscope bias estimation, separate hit detectors for each hand, return-travel and inter-hit safeguards, three orientation regions, multi-voice audio playback and session logging. Plotting was removed from the real-time path because graphical updates introduced perceptible stalls.

The latest implementation moved acquisition into Arduino firmware. Both IMUs and all four EMG envelope channels are now sampled into one timestamped binary frame at 200 Hz. Each EMG channel is normalised using its own rest level and active reference, which compensates for the different amplifier gains. The IMU proposes the hit and drum label; EMG acts as an independent confirmation rather than generating a synthetic hit.

## 3. Experimental evidence

Table 1 compares representative IMU-only sessions with the latest synchronised IMU–EMG session. All values were calculated from the saved MAT files. The recordings contain system predictions but not an independent label for every intended live hit. Consequently, the reported acceptance rate must not be interpreted as classification accuracy.

**Table 1: Acquisition and hit-processing comparison**

| Session | Configuration | Duration (s) | Sample rate (Hz) | Sample interval (ms) | IMU candidates | Accepted hits | Acceptance rate | Invalid frames |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 20261011_122220 | IMU only | 281.743 | 15.579 | 64.19 | 370 | 370* | 100%* | 0 |
| 20261011_125325 | IMU only | 167.301 | 15.116 | 66.16 | 220 | 220* | 100%* | 0 |
| 20261011_152111 | IMU + EMG | 234.735 | 200.000 | 5.00 | 199 | 176 | 88.4% | 0 |
| 20261011_154714 | IMU + EMG | 92.250 | 200.000 | 5.00 | 40 | 32 | 80.0% | 0 |
| 20261011_174814 | IMU + EMG | 157.365 | 200.000 | 5.00 | 132 | 125 | 94.7% | 0 |

\*The IMU-only implementation had no independent EMG rejection stage, so every detector output was treated as a hit. This is not evidence that every output represented an intentional strike.

The synchronised implementation increased the measured sampling rate by a factor of approximately 13.2 relative to session 20261011_125325. The sample interval decreased from 66.16 ms to 5.00 ms, a reduction of approximately 92.4%. Earlier IMU-only sessions also exhibited occasional loop stalls: the maximum recorded loop duration was 1.696 s in session 20261011_122220. In the synchronised sessions, device timestamps remained uniformly spaced at 5.00 ms and no invalid frames or non-zero device-status frames were recorded.

### 3.1 Orientation behaviour

The fraction of IMU hits assigned a non-zero drum region improved markedly between the two representative IMU-only sessions. Session 20261011_122220 assigned 137 of 370 hits (37.0%), whereas session 20261011_125325 assigned 199 of 220 hits (90.5%). This demonstrates improved orientation availability, but the variation between sessions also confirms that orientation calibration and playing posture remain material sources of uncertainty.

For the latest IMU–EMG session, the candidate classification matrix was:

**Table 2: Latest candidate drum labels (session 20261011_174814)**

| Hand | Uncertain | Kick | Snare | Crash | Total |
|---|---:|---:|---:|---:|---:|
| Left | 1 | 35 | 16 | 10 | 62 |
| Right | 4 | 53 | 0 | 13 | 70 |
| **Total** | **5** | **88** | **16** | **23** | **132** |

Only 5 of 132 candidates (3.8%) were rejected because their orientation was uncertain. However, the absence of right-hand snare predictions may reflect the performed gestures, an orientation-region bias, or both. Intended labels are required to distinguish these explanations.

### 3.2 EMG confirmation and delay

**Table 3: Latest per-hand fusion performance**

| Hand | IMU candidates | EMG-confirmed hits | Confirmation rate | Median EMG score | Minimum candidate gap (ms) |
|---|---:|---:|---:|---:|---:|
| Left | 62 | 60 | 96.8% | 3.249 | 285 |
| Right | 70 | 65 | 92.9% | 1.543 | 295 |
| **Combined** | **132** | **125** | **94.7%** | — | — |

The decision matrix for session 20261011_174814 contained 125 accepted candidates, two EMG timeouts and five uncertain-orientation rejections. The median and 95th-percentile recorded decision delays were both 0 ms because EMG activity was already present inside the 180 ms look-back window when the IMU candidate occurred. The true sensor-side timing resolution is 5 ms. These figures do not include MATLAB audio scheduling, operating-system buffering or Bluetooth-speaker latency, so they are classifier-delay measurements rather than complete acoustic latency measurements.

The EMG channels reached usable amplitudes after independent normalisation. However, the two right-arm channels had a correlation coefficient of 0.9981, compared with 0.6808 between the two left-arm channels. The right forearm and triceps channels therefore supplied almost identical information in this session. Electrode placement, shared motion artefact, grounding and amplifier gain should be checked before these channels are used to distinguish gestures.

## 4. IMU-only versus IMU–EMG behaviour

The IMU-only system produced immediate sounds and detected alternating and simultaneous strokes, but it could not distinguish an intentional strike from a similar unintentional acceleration. Its low sequential read rate also limited temporal resolution and occasionally produced long stalls. Anti-double-trigger state and independent per-hand timers reduced repeated sounds without preventing simultaneous two-hand operation.

The synchronised IMU–EMG system preserved the IMU as the primary motion and drum-region sensor while adding physiological confirmation. This architecture can reject candidates that lack corresponding muscle activity and accommodates different analogue gains through per-channel normalisation. Its 200 Hz shared timestamp also removes the timing ambiguity caused by sequential MATLAB reads. The trade-off is that weak electrodes or an incorrectly selected EMG threshold can reject a genuine hit. In the latest session, two candidates timed out without sufficient EMG evidence.

## 5. Preliminary feasibility conclusion

The measured results support the preliminary feasibility of a two-handed, three-sound virtual drum system. The acquisition rate, frame validity and confirmation rate are suitable for continued prototype investigation, and the latest fusion classifier added no measurable algorithmic delay beyond the 5 ms sampling resolution for most hits. Nevertheless, feasibility has not yet been demonstrated in terms of drum-selection accuracy, false-positive rate or end-to-end acoustic latency. Those quantities require independently labelled intended hits and measured sound-output timing.

The next experiment should collect guided, round-labelled kick, snare and crash strikes for both hands, together with non-hit movement, alternating patterns and simultaneous hits. Training and validation must be separated by complete rounds or sessions. The resulting evaluation should report a confusion matrix, per-class precision and recall, missed-hit and double-trigger rates, false positives per minute, simultaneous-hit success, and median and 95th-percentile acoustic latency. The right-arm EMG electrode arrangement should be corrected before that dataset is collected.

## 6. Limitations

This report is based on a small number of sessions performed by one user. The live recordings do not contain independent intended-drum labels, and therefore no defensible classification-accuracy matrix can yet be calculated. The 100% IMU-only values in Table 1 indicate the absence of a confirmation stage, not perfect detection. Bluetooth and host-audio latency were not instrumented. These limitations prevent a final robustness claim, but they define the measurements required for the next phase.
