# Machine-learning classifier

This folder owns feature preparation, model training, grouped validation,
metrics, saved-model metadata, and inference. Preprocessing must be fitted only
on training data. Trials from the same collection round/session must not be
silently split across training and evaluation sets.

`analyzeDrumMapData.m` is retained from the previous five-drum investigation.
Its current results are a baseline, not a production model.

Run `extractMLFeatureDataset` after completing multiple labelled collections.
It locates the requested number of movement peaks inside each hit block,
selects evenly distributed windows from `NO_HIT` blocks, and writes the
combined table to `data/processed/machineLearning`. Session and round columns
are retained for leakage-free grouped validation and are not predictors.

`trainMLBaggedTreeClassifier` trains a bagged-tree ensemble and evaluates it
with leave-one-session-out validation. Every reported prediction therefore
comes from a model that did not see any window from that recording session.
The final all-session model, confusion matrix, per-class metrics, held-out
predictions and feature importance are saved under `data/models/machineLearning`.

`initializeLiveMLClassifier` validates and loads the saved artifact.
`updateLiveMLClassifier` buffers the 0.20-second pre-candidate and 0.10-second
post-candidate window, calculates the same 60 features, and predicts `NO_HIT`,
`KICK`, `SNARE` or `CRASH`. Live predictions require at least 50% confidence
and both same-hand EMG channels. Hybrid mode also requires agreement with the
rule-based gravity-region label. Isolated invalid IMU frames are discarded as
long as at least 80% of the expected feature window remains.
