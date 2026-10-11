function model = buildAzimuthModel(azimuthDeg, timeSeconds, handIndex, ...
        labels, targetNames, options)
% BUILDAZIMUTHMODEL Fit per-hand drum azimuths while removing linear drift.
%
% Calibration hits are collected interleaved (SNARE, KICK, CRASH, KICK, ...)
% so every drum is visited throughout the calibration. That makes a common
% linear drift identifiable: azimuth = centre(label) + drift * (t - tRef).
% The drift is fitted per hand and removed, so the stored centres describe
% where each drum is at the END of calibration, which is where play begins.
%
%   model = buildAzimuthModel(az, t, hand, labels, ["SNARE" "KICK" "CRASH"], ...
%       MinSpreadDeg = 6)

    arguments
        azimuthDeg (:,1) double {mustBeFinite}
        timeSeconds (:,1) double {mustBeFinite}
        handIndex (:,1) double {mustBeInteger, mustBeInRange(handIndex, 1, 2)}
        labels (:,1) string
        targetNames (1,:) string
        options.MinSpreadDeg (1,1) double {mustBePositive} = 6
        options.MinSeparationDeg (1,1) double {mustBePositive} = 15
        options.MinAccuracy (1,1) double {mustBePositive} = 0.80
        options.MaxDriftDegPerSec (1,1) double {mustBePositive} = 3
    end

    count = numel(azimuthDeg);
    if numel(timeSeconds) ~= count || numel(handIndex) ~= count || ...
            numel(labels) ~= count
        error('MyoDrum:InvalidAzimuthCalibrationData', ...
            'Azimuth, time, hand and label vectors need equal lengths.');
    end
    targetCount = numel(targetNames);
    model.targetNames = targetNames;
    model.hands = repmat(struct( ...
        'centersDeg', nan(1, targetCount), ...
        'spreadDeg', nan(1, targetCount), ...
        'driftDegPerSec', 0, 'sampleCount', zeros(1, targetCount), ...
        'minSeparationDeg', NaN, 'selfAccuracy', NaN, ...
        'ok', false, 'reason', ""), 1, 2);

    for hand = 1:2
        rows = find(handIndex == hand);
        h = model.hands(hand);
        missing = false;
        for k = 1:targetCount
            h.sampleCount(k) = sum(labels(rows) == targetNames(k));
            missing = missing || h.sampleCount(k) < 2;
        end
        if missing
            h.reason = "needs at least 2 hits per drum";
            model.hands(hand) = h;
            continue;
        end

        a = azimuthDeg(rows);
        t = timeSeconds(rows);
        lab = labels(rows);
        tRef = max(t);

        % Circular mean per label as a reference, then a linear fit of the
        % wrapped residuals: residual = offset(label) + drift * (t - tRef).
        circularMean = zeros(1, targetCount);
        for k = 1:targetCount
            members = a(lab == targetNames(k));
            circularMean(k) = atan2d(mean(sind(members)), mean(cosd(members)));
        end
        labelIndex = zeros(numel(a), 1);
        for k = 1:targetCount
            labelIndex(lab == targetNames(k)) = k;
        end
        residual = wrapAngle(a - circularMean(labelIndex).');
        design = zeros(numel(a), targetCount + 1);
        for k = 1:targetCount
            design(:, k) = double(labelIndex == k);
        end
        design(:, end) = t - tRef;
        % Robust (Huber-style) reweighting so one wild hit cannot bend the
        % drift estimate.
        weights = ones(numel(a), 1);
        coefficients = zeros(targetCount + 1, 1);
        for iteration = 1:6
            sqrtW = sqrt(weights);
            coefficients = (design .* sqrtW) \ (residual .* sqrtW);
            fitted = design * coefficients;
            error = residual - fitted;
            scale = max(1.4826 * median(abs(error)), 3);
            weights = min(1, 1.5 * scale ./ max(abs(error), eps));
        end
        drift = coefficients(end);
        if numel(unique(t)) < 4 || abs(drift) > options.MaxDriftDegPerSec
            % Not identifiable (or implausible): fall back to no drift.
            coefficients(end) = 0;
            drift = 0;
            fitted = design * coefficients;
            error = residual - fitted;
        end
        h.driftDegPerSec = drift;
        for k = 1:targetCount
            h.centersDeg(k) = wrapAngle(circularMean(k) + coefficients(k));
            members = error(labelIndex == k);
            h.spreadDeg(k) = max(1.4826 * median(abs(members)), ...
                options.MinSpreadDeg);
        end

        % Minimum circular separation between drum centres.
        separations = [];
        for i = 1:targetCount
            for j = i + 1:targetCount
                separations(end + 1) = abs(wrapAngle( ...
                    h.centersDeg(i) - h.centersDeg(j))); %#ok<AGROW>
            end
        end
        h.minSeparationDeg = min(separations);

        % Self-accuracy with the drift removed, using the playing rule.
        correct = 0;
        for row = 1:numel(a)
            corrected = wrapAngle(a(row) - drift * (t(row) - tRef));
            distances = abs(wrapAngle(corrected - h.centersDeg)) ./ ...
                h.spreadDeg;
            [~, nearest] = min(distances);
            correct = correct + double(nearest == labelIndex(row));
        end
        h.selfAccuracy = correct / numel(a);

        if h.minSeparationDeg < options.MinSeparationDeg
            h.reason = sprintf('two drums only %.0f deg apart (need %.0f)', ...
                h.minSeparationDeg, options.MinSeparationDeg);
        elseif h.selfAccuracy < options.MinAccuracy
            h.reason = sprintf('self-accuracy %.0f%% (need %.0f%%)', ...
                100 * h.selfAccuracy, 100 * options.MinAccuracy);
        else
            h.ok = true;
            h.reason = "ok";
        end
        model.hands(hand) = h;
    end
    model.createdAt = string(datetime('now', 'TimeZone', 'local'));
    model.schemaVersion = "azimuth-1";
end

function wrapped = wrapAngle(angleDeg)
    wrapped = mod(angleDeg + 180, 360) - 180;
end
