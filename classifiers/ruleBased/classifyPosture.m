function [label, confidence, diagnostics] = classifyPosture( ...
        model, handIndex, gravityVector)
% CLASSIFYPOSTURE Choose the hand's default drum or the crash posture.
%
% The detected hand decides the ordinary sound. Only a posture that is
% clearly closer to the calibrated crash posture than to that hand's normal
% posture, and inside the crash region, plays CRASH. If the crash posture
% was not separable during calibration (crashEnabled false), the hand
% always plays its default drum rather than guessing.

    arguments
        model (1,1) struct
        handIndex (1,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        gravityVector (1,3) double {mustBeFinite}
    end

    h = model.hands(handIndex);
    label = model.handDrumNames(handIndex);
    confidence = 1;
    diagnostics.crashEnabled = h.crashEnabled;
    diagnostics.anglesDeg = [NaN, NaN];
    diagnostics.normalized = [NaN, NaN];

    length = norm(gravityVector);
    if length < 0.5
        return;
    end
    unit = gravityVector / length;
    angles = acosd(max(-1, min(1, h.centers * unit(:)))).';
    normalized = angles ./ h.spreadDeg;
    diagnostics.anglesDeg = angles;
    diagnostics.normalized = normalized;
    if ~h.crashEnabled
        return;
    end
    if normalized(2) <= model.maxNormalizedDistance && ...
            normalized(2) < normalized(1) - model.marginNormalized
        label = model.crashName;
        confidence = (normalized(1) - normalized(2)) / ...
            max(normalized(1) + normalized(2), eps);
    end
end
