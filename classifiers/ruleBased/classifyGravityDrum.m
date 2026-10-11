function [label, confidence, diagnostics] = classifyGravityDrum( ...
        model, handIndex, gravityVector, options)
% CLASSIFYGRAVITYDRUM Select a drum pose and optionally validate its swing.
%
% Drum identity comes only from the pre-stroke gravity direction. A peak
% gyro vector can reject a stroke whose rotational direction is inconsistent
% with the calibrated motion, but it never changes one drum into another.

    arguments
        model (1,1) struct
        handIndex (1,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        gravityVector (1,3) double {mustBeFinite}
        options.MaximumDistance (1,1) double {mustBePositive} = 3
        options.AmbiguityMargin (1,1) double {mustBeNonnegative} = 0.35
        options.CurrentLabel (1,1) string = ""
        options.ExitDistance (1,1) double {mustBePositive} = 3.75
        options.GyroPeakVector (1,3) double {mustBeFinite} = [0 0 0]
        options.MaximumGyroAngleDeg (1,1) double ...
            {mustBeInRange(options.MaximumGyroAngleDeg, 0, 180)} = 80
    end

    gravityNorm = norm(gravityVector);
    if gravityNorm < 0.5
        error('MyoDrum:InvalidGravityVector', ...
            'Gravity vector must have a non-zero magnitude.');
    end
    gravity = gravityVector / gravityNorm;
    targetCount = numel(model.targetNames);
    distances = zeros(1, targetCount);
    gravityAngles = zeros(1, targetCount);
    for target = 1:targetCount
        center = reshape(model.gravityCenters(handIndex, target, :), 1, 3);
        gravityAngles(target) = vectorAngle(gravity, center);
        distances(target) = gravityAngles(target) / ...
            model.gravitySpreadDeg(handIndex, target);
    end
    [ordered, order] = sort(distances);
    accepted = ordered(1) <= options.MaximumDistance;
    ambiguous = targetCount > 1 && ...
        ordered(2) - ordered(1) < options.AmbiguityMargin;
    currentIndex = find(model.targetNames == options.CurrentLabel, 1);
    retained = ~isempty(currentIndex) && ...
        distances(currentIndex) <= options.ExitDistance && ...
        (order(1) == currentIndex || ~accepted || ambiguous);
    if retained
        selected = currentIndex;
    elseif accepted && ~ambiguous
        selected = order(1);
    else
        selected = [];
    end

    gyroAccepted = true;
    gyroAngle = NaN;
    if ~isempty(selected) && norm(options.GyroPeakVector) > 1e-6
        gyroDirection = options.GyroPeakVector / norm(options.GyroPeakVector);
        gyroCenter = reshape(model.gyroCenters( ...
            handIndex, selected, :), 1, 3);
        gyroAngle = vectorAngle(gyroDirection, gyroCenter);
        gyroAccepted = gyroAngle <= options.MaximumGyroAngleDeg;
    end
    if isempty(selected) || ~gyroAccepted
        label = "";
        confidence = 0;
    else
        label = model.targetNames(selected);
        confidence = max(0, 1 - distances(selected) / ...
            options.MaximumDistance);
    end
    diagnostics.gravityAnglesDeg = gravityAngles;
    diagnostics.normalizedDistances = distances;
    diagnostics.ambiguous = ambiguous;
    diagnostics.retainedByHysteresis = retained;
    diagnostics.gyroAngleDeg = gyroAngle;
    diagnostics.gyroAccepted = gyroAccepted;
end

function angle = vectorAngle(a, b)
    angle = acosd(max(-1, min(1, dot(a, b))));
end
