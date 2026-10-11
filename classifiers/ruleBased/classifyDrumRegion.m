function [label, confidence, diagnostics] = classifyDrumRegion( ...
        model, handIndex, orientationDeg, maximumDistance, ambiguityMargin, ...
        currentLabel, exitDistance)
% CLASSIFYDRUMREGION Select the nearest calibrated orientation region.
%
% currentLabel and exitDistance provide real hysteresis: an active region
% remains selected until the measurement leaves its wider exit boundary.

    arguments
        model (1,1) struct
        handIndex (1,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        orientationDeg (1,2) double {mustBeFinite}
        maximumDistance (1,1) double {mustBeFinite, mustBePositive} = 3
        ambiguityMargin (1,1) double {mustBeFinite, mustBeNonnegative} = 0.35
        currentLabel (1,1) string = ""
        exitDistance (1,1) double {mustBeFinite, mustBePositive} = 3.75
    end
    targetCount = numel(model.targetNames);
    distances = zeros(1, targetCount);
    for targetIndex = 1:targetCount
        center = reshape(model.centersDeg(handIndex, targetIndex, :), 1, 2);
        spread = reshape(model.spreadsDeg(handIndex, targetIndex, :), 1, 2);
        residual = orientationDeg - center;
        if isfield(model, 'featureNames')
            circular = model.featureNames == "Yaw" | ...
                model.featureNames == "Roll";
            residual(circular) = wrapDegrees(residual(circular));
        else
            residual(1) = wrapDegrees(residual(1));
        end
        distances(targetIndex) = norm(residual ./ spread);
    end
    [ordered, order] = sort(distances);
    accepted = ordered(1) <= maximumDistance;
    ambiguous = targetCount > 1 && ...
        (ordered(2) - ordered(1)) < ambiguityMargin;
    currentIndex = find(model.targetNames == currentLabel, 1);
    retainedByHysteresis = ~isempty(currentIndex) && ...
        distances(currentIndex) <= exitDistance && ...
        (order(1) == currentIndex || ~accepted || ambiguous);
    if retainedByHysteresis
        label = currentLabel;
        confidence = max(0, 1 - distances(currentIndex) / exitDistance);
        diagnostics.distances = distances;
        diagnostics.accepted = true;
        diagnostics.ambiguous = false;
        diagnostics.retainedByHysteresis = true;
        return;
    end
    if accepted && ~ambiguous
        label = model.targetNames(order(1));
        confidence = max(0, 1 - ordered(1) / maximumDistance);
    else
        label = "";
        confidence = 0;
    end
    diagnostics.distances = distances;
    diagnostics.accepted = accepted;
    diagnostics.ambiguous = ambiguous;
    diagnostics.retainedByHysteresis = false;
end

function wrapped = wrapDegrees(angleDeg)
    wrapped = mod(angleDeg + 180, 360) - 180;
end
