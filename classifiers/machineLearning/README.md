# Machine-learning classifier

This folder owns feature preparation, model training, grouped validation,
metrics, saved-model metadata, and inference. Preprocessing must be fitted only
on training data. Trials from the same collection round/session must not be
silently split across training and evaluation sets.

`analyzeDrumMapData.m` is retained from the previous five-drum investigation.
Its current results are a baseline, not a production model.
