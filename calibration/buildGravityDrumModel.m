function model = buildGravityDrumModel(gravityVectors, gyroPeakVectors, ...
        handIndex, labels, targetNames, options)
% BUILDGRAVITYDRUMMODEL Fit per-hand drum poses and stroke directions.
%
% Gravity is represented directly as a unit 3-D vector. This avoids the
% pitch +/-90 degree singularity and roll wrap that occur with Euler angles.
% The peak gyro vector is a secondary consistency check, not the drum label.

    arguments
        gravityVectors (:,3) double {mustBeFinite}
        gyroPeakVectors (:,3) double {mustBeFinite}
        handIndex (:,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        labels (:,1) string
        targetNames (1,:) string
        options.MinGravitySpreadDeg (1,1) double {mustBePositive} = 6
        options.MinGyroSpreadDeg (1,1) double {mustBePositive} = 12
        options.MinSeparationDeg (1,1) double {mustBePositive} = 20
        options.MinTrainingAccuracy (1,1) double {mustBeInRange( ...
            options.MinTrainingAccuracy, 0, 1)} = 0.80
    end

    count = size(gravityVectors, 1);
    if size(gyroPeakVectors, 1) ~= count || numel(handIndex) ~= count || ...
            numel(labels) ~= count
        error('MyoDrum:InvalidGravityDrumCalibrationData', ...
            'Gravity, gyro, hand and label inputs need equal row counts.');
    end
    gravityVectors = normalizeRows(gravityVectors, "gravity");
    gyroPeakVectors = normalizeRows(gyroPeakVectors, "gyro");
    targetCount = numel(targetNames);
    model.targetNames = targetNames;
    model.gravityCenters = nan(2, targetCount, 3);
    model.gravitySpreadDeg = nan(2, targetCount);
    model.gyroCenters = nan(2, targetCount, 3);
    model.gyroSpreadDeg = nan(2, targetCount);
    model.sampleCount = zeros(2, targetCount);
    model.trainingAccuracyByHand = zeros(1, 2);
    model.minimumSeparationDegByHand = nan(1, 2);
    model.okByHand = false(1, 2);

    for hand = 1:2
        for target = 1:targetCount
            selected = handIndex == hand & labels == targetNames(target);
            if sum(selected) < 2
                error('MyoDrum:MissingGravityDrumCalibrationClass', ...
                    'Hand %d target %s needs at least two calibration hits.', ...
                    hand, targetNames(target));
            end
            model.sampleCount(hand, target) = sum(selected);
            gravityCenter = normalizedMean(gravityVectors(selected, :));
            gyroCenter = normalizedMean(gyroPeakVectors(selected, :));
            gravityAngles = angleBetween(gravityVectors(selected, :), ...
                gravityCenter);
            gyroAngles = angleBetween(gyroPeakVectors(selected, :), gyroCenter);
            model.gravityCenters(hand, target, :) = gravityCenter;
            model.gyroCenters(hand, target, :) = gyroCenter;
            model.gravitySpreadDeg(hand, target) = max( ...
                1.4826 * median(gravityAngles), options.MinGravitySpreadDeg);
            model.gyroSpreadDeg(hand, target) = max( ...
                1.4826 * median(gyroAngles), options.MinGyroSpreadDeg);
        end

        separation = zeros(0, 1);
        for first = 1:targetCount
            for second = first + 1:targetCount
                a = reshape(model.gravityCenters(hand, first, :), 1, 3);
                b = reshape(model.gravityCenters(hand, second, :), 1, 3);
                separation(end + 1, 1) = angleBetween(a, b); %#ok<AGROW>
            end
        end
        model.minimumSeparationDegByHand(hand) = min(separation);

        rows = find(handIndex == hand);
        correct = 0;
        for row = rows.'
            distances = gravityDistances(model, hand, gravityVectors(row, :));
            [~, nearest] = min(distances);
            correct = correct + double(targetNames(nearest) == labels(row));
        end
        model.trainingAccuracyByHand(hand) = correct / numel(rows);
        model.okByHand(hand) = ...
            model.minimumSeparationDegByHand(hand) >= ...
                options.MinSeparationDeg && ...
            model.trainingAccuracyByHand(hand) >= ...
                options.MinTrainingAccuracy;
    end
    model.createdAt = string(datetime('now', 'TimeZone', 'local'));
    model.schemaVersion = "gravity-vector-drums-1";
end

function normalized = normalizeRows(values, name)
    lengths = vecnorm(values, 2, 2);
    if any(lengths < 1e-6)
        error('MyoDrum:InvalidGravityDrumCalibrationData', ...
            'Every %s calibration vector must be non-zero.', name);
    end
    normalized = values ./ lengths;
end

function center = normalizedMean(values)
    center = mean(values, 1);
    if norm(center) < 0.1
        error('MyoDrum:InconsistentGravityDrumCalibration', ...
            'Calibration vectors point in inconsistent directions.');
    end
    center = center / norm(center);
end

function angles = angleBetween(vectors, reference)
    angles = acosd(max(-1, min(1, vectors * reference(:))));
end

function distances = gravityDistances(model, hand, gravity)
    targetCount = numel(model.targetNames);
    distances = zeros(1, targetCount);
    for target = 1:targetCount
        center = reshape(model.gravityCenters(hand, target, :), 1, 3);
        distances(target) = angleBetween(gravity, center) / ...
            model.gravitySpreadDeg(hand, target);
    end
end
