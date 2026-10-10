# Rule-based classifier

This folder will contain interpretable drum-target rules and thresholds. The
classifier must accept documented features and return a target, confidence, and
ambiguity status without depending on hardware access or audio playback.

The preserved kick-only state machine currently provides the verified event
detection baseline; it remains in `drumRunTests/kickOnly` until its reusable
parts are extracted without changing its observed behaviour.
