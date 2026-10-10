# Experiment protocol outline

The primary comparisons are:

1. Rule-based versus machine-learning target classification.
2. IMU-only versus synchronized IMU+EMG input.

Use the same target set, trial definitions, feature versions, and held-out test
trials for fair comparisons. Split data by collection round or session rather
than randomly splitting neighbouring hits from the same recording. Report the
confusion matrix, per-class precision/recall/F1, overall accuracy, rejected or
ambiguous trials, missed/duplicate hits, and inference latency.

The first gate is a three-target pilot. Do not expand to five targets until
held-out-round performance and live behaviour are acceptably repeatable.
