function [confirmed, score, normalized] = confirmEMGHit( ...
        recentVoltage, handName, profile, threshold)
% CONFIRMEMGHIT Confirm an IMU hit from independently normalized EMG.

    arguments
        recentVoltage (:,:) double {mustBeFinite}
        handName (1,1) string
        profile (1,1) struct
        threshold (1,1) double {mustBeNonnegative}
    end
    peak = max(recentVoltage, [], 1);
    normalized = max(0, (peak - profile.baseline_V) ./ ...
        profile.activeSpan_V);
    eligible = profile.hands == handName & profile.validChannel(:);
    if ~any(eligible)
        confirmed = false;
        score = 0;
        return;
    end
    score = max(normalized(eligible));
    confirmed = score >= threshold;
end
