function model = buildDrumRegionModel(orientationFeaturesDeg, handIndex, labels, ...
        targetNames, minimumSpreadDeg, minimumSeparationDeg, ...
        minimumTrainingAccuracy)
% BUILDDRUMREGIONMODEL Fit hand-specific drift-free pitch/roll regions.

    arguments
        orientationFeaturesDeg (:,2) double {mustBeFinite}
        handIndex (:,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        labels (:,1) string
        targetNames (1,:) string
        minimumSpreadDeg (1,2) double {mustBeFinite, mustBePositive} = [5, 5]
        minimumSeparationDeg (1,1) double {mustBeFinite, mustBePositive} = 15
        minimumTrainingAccuracy (1,1) double {mustBeInRange( ...
            minimumTrainingAccuracy, 0, 1)} = 0.80
    end
    sampleCount = size(orientationFeaturesDeg, 1);
    if numel(handIndex) ~= sampleCount || numel(labels) ~= sampleCount
        error('MyoDrum:InvalidRegionCalibrationData', ...
            'Angles, hand indices, and labels must have equal row counts.');
    end

    targetCount = numel(targetNames);
    model.targetNames = targetNames;
    model.featureNames = ["Pitch", "Roll"];
    model.centersDeg = nan(2, targetCount, 2);
    model.spreadsDeg = nan(2, targetCount, 2);
    model.sampleCount = zeros(2, targetCount);
    for hand = 1:2
        for targetIndex = 1:targetCount
            selected = handIndex == hand & labels == targetNames(targetIndex);
            if ~any(selected)
                error('MyoDrum:MissingRegionCalibrationClass', ...
                    'No calibration hits exist for hand %d target %s.', ...
                    hand, targetNames(targetIndex));
            end
            values = orientationFeaturesDeg(selected, :);
            pitchCenter = median(values(:, 1));
            rollRadians = deg2rad(values(:, 2));
            rollCenter = rad2deg(atan2(mean(sin(rollRadians)), ...
                mean(cos(rollRadians))));
            pitchResidual = values(:, 1) - pitchCenter;
            rollResidual = wrapDegrees(values(:, 2) - rollCenter);
            robustSpread = 1.4826 * median(abs( ...
                [pitchResidual, rollResidual]), 1);
            model.centersDeg(hand, targetIndex, :) = ...
                [pitchCenter, rollCenter];
            model.spreadsDeg(hand, targetIndex, :) = ...
                max(robustSpread, minimumSpreadDeg);
            model.sampleCount(hand, targetIndex) = sum(selected);
        end
    end
    model.trainingAccuracyByHand = zeros(1, 2);
    model.minimumSeparationDegByHand = nan(1, 2);
    model.okByHand = false(1, 2);
    for hand = 1:2
        selectedHand = handIndex == hand;
        predicted = strings(sum(selectedHand), 1);
        handFeatures = orientationFeaturesDeg(selectedHand, :);
        for sampleIndex = 1:size(handFeatures, 1)
            distances = zeros(1, targetCount);
            for targetIndex = 1:targetCount
                center = reshape(model.centersDeg( ...
                    hand, targetIndex, :), 1, 2);
                spread = reshape(model.spreadsDeg( ...
                    hand, targetIndex, :), 1, 2);
                residual = handFeatures(sampleIndex, :) - center;
                residual(2) = wrapDegrees(residual(2));
                distances(targetIndex) = norm(residual ./ spread);
            end
            [~, nearest] = min(distances);
            predicted(sampleIndex) = targetNames(nearest);
        end
        expected = labels(selectedHand);
        model.trainingAccuracyByHand(hand) = mean(predicted == expected);
        separations = zeros(0, 1);
        for first = 1:targetCount
            for second = first + 1:targetCount
                firstCenter = reshape(model.centersDeg( ...
                    hand, first, :), 1, 2);
                secondCenter = reshape(model.centersDeg( ...
                    hand, second, :), 1, 2);
                difference = firstCenter - secondCenter;
                difference(2) = wrapDegrees(difference(2));
                separations(end + 1, 1) = norm(difference); %#ok<AGROW>
            end
        end
        model.minimumSeparationDegByHand(hand) = min(separations);
        model.okByHand(hand) = ...
            model.minimumSeparationDegByHand(hand) >= minimumSeparationDeg && ...
            model.trainingAccuracyByHand(hand) >= minimumTrainingAccuracy;
    end
    model.createdAt = string(datetime('now', 'TimeZone', 'local'));
    model.schemaVersion = "gravity-regions-1";
end

function wrapped = wrapDegrees(angleDeg)
    wrapped = mod(angleDeg + 180, 360) - 180;
end
