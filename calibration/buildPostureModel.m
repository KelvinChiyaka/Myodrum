function model = buildPostureModel(gravityVectors, handIndex, labels, ...
        handDrumNames, crashName, options)
% BUILDPOSTUREMODEL Fit a drift-free two-class posture model per hand.
%
% Each hand has a default drum (its hand identity decides the sound) and one
% alternative posture (crash). Both are described by the gravity direction
% in the sensor frame at the start of the downstroke. Gravity is an absolute
% reference, so unlike yaw this model cannot drift or rotate around the
% player.
%
%   model = buildPostureModel(gravityVectors, handIndex, labels, ...
%       ["SNARE" "KICK"], "CRASH", options)
%
% options.MinSpreadDeg, options.MinSeparationDeg, options.MinTrainingAccuracy,
% options.MaxNormalizedDistance, options.MarginNormalized

    arguments
        gravityVectors (:,3) double {mustBeFinite}
        handIndex (:,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        labels (:,1) string
        handDrumNames (1,2) string
        crashName (1,1) string
        options.MinSpreadDeg (1,1) double {mustBePositive} = 4
        options.MinSeparationDeg (1,1) double {mustBePositive} = 20
        options.MinTrainingAccuracy (1,1) double {mustBePositive} = 0.85
        options.MaxNormalizedDistance (1,1) double {mustBePositive} = 3
        options.MarginNormalized (1,1) double {mustBeNonnegative} = 0.25
    end

    sampleCount = size(gravityVectors, 1);
    if numel(handIndex) ~= sampleCount || numel(labels) ~= sampleCount
        error('MyoDrum:InvalidPostureCalibrationData', ...
            'Gravity vectors, hand indices and labels need equal row counts.');
    end
    norms = vecnorm(gravityVectors, 2, 2);
    if any(norms < 0.5)
        error('MyoDrum:InvalidPostureCalibrationData', ...
            'Every gravity vector must have a non-zero length.');
    end
    gravityVectors = gravityVectors ./ norms;

    model.handDrumNames = handDrumNames;
    model.crashName = crashName;
    model.minSpreadDeg = options.MinSpreadDeg;
    model.maxNormalizedDistance = options.MaxNormalizedDistance;
    model.marginNormalized = options.MarginNormalized;
    model.hands = repmat(struct( ...
        'centers', zeros(2, 3), 'spreadDeg', zeros(1, 2), ...
        'sampleCount', zeros(1, 2), 'separationDeg', NaN, ...
        'trainingAccuracy', NaN, 'crashEnabled', false, ...
        'reason', ""), 1, 2);

    for hand = 1:2
        classNames = [handDrumNames(hand), crashName];
        for classIndex = 1:2
            selected = handIndex == hand & labels == classNames(classIndex);
            count = sum(selected);
            model.hands(hand).sampleCount(classIndex) = count;
            if count == 0
                model.hands(hand).reason = "missing calibration hits for " + ...
                    classNames(classIndex);
                continue;
            end
            members = gravityVectors(selected, :);
            center = mean(members, 1);
            center = center / norm(center);
            angles = angleBetween(members, center);
            spread = max(1.4826 * median(angles), options.MinSpreadDeg);
            model.hands(hand).centers(classIndex, :) = center;
            model.hands(hand).spreadDeg(classIndex) = spread;
        end
        if any(model.hands(hand).sampleCount == 0)
            continue;
        end
        centers = model.hands(hand).centers;
        model.hands(hand).separationDeg = angleBetween(centers(1, :), centers(2, :));

        % Self-accuracy with the same decision rule used while playing.
        selectedHand = handIndex == hand & ...
            (labels == classNames(1) | labels == classNames(2));
        correct = 0;
        total = 0;
        for row = find(selectedHand).'
            predicted = decide(model, hand, gravityVectors(row, :), true);
            correct = correct + double(predicted == labels(row));
            total = total + 1;
        end
        model.hands(hand).trainingAccuracy = correct / max(total, 1);

        if model.hands(hand).separationDeg < options.MinSeparationDeg
            model.hands(hand).reason = sprintf( ...
                'crash posture only %.0f deg from normal (need %.0f)', ...
                model.hands(hand).separationDeg, options.MinSeparationDeg);
        elseif model.hands(hand).trainingAccuracy < options.MinTrainingAccuracy
            model.hands(hand).reason = sprintf( ...
                'self-accuracy %.0f%% (need %.0f%%)', ...
                100 * model.hands(hand).trainingAccuracy, ...
                100 * options.MinTrainingAccuracy);
        else
            model.hands(hand).crashEnabled = true;
            model.hands(hand).reason = "ok";
        end
    end
    model.createdAt = string(datetime('now', 'TimeZone', 'local'));
    model.schemaVersion = "posture-1";
end

function label = decide(model, hand, gravityVector, ignoreEnabled)
    h = model.hands(hand);
    label = model.handDrumNames(hand);
    if ~ignoreEnabled && ~h.crashEnabled
        return;
    end
    angles = angleBetween(h.centers, gravityVector);
    normalized = angles(:).' ./ h.spreadDeg;
    if normalized(2) <= model.maxNormalizedDistance && ...
            normalized(2) < normalized(1) - model.marginNormalized
        label = model.crashName;
    end
end

function angles = angleBetween(vectors, reference)
    dotProducts = vectors * reference(:);
    angles = acosd(max(-1, min(1, dotProducts)));
end
