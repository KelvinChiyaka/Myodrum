# EMG subsystem

This folder is reserved for verified EMG acquisition and preprocessing code.
The subsystem will provide channel mapping, filtering, rest baseline,
session-specific normalization, signal-quality flags, and activation/timing
features.

EMG is currently disabled in `common/myoDrumConfig.m`. No synthetic EMG values
should be substituted when the hardware is unavailable.
