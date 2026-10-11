function profile = buildEMGConfirmationProfile( ...
        restVoltage, activeVoltage, hands, options)
% BUILDEMGCONFIRMATIONPROFILE Normalize independently gained EMG channels.

    arguments
        restVoltage (:,:) double {mustBeFinite}
        activeVoltage (:,:) double {mustBeFinite}
        hands (:,1) string
        options.MinimumActiveSpanVolts (1,1) double {mustBePositive} = 0.10
        options.ActivePercentile (1,1) double ...
            {mustBeInRange(options.ActivePercentile, 50, 100)} = 90
    end
    channelCount = numel(hands);
    if size(restVoltage, 2) ~= channelCount || ...
            size(activeVoltage, 2) ~= channelCount
        error('MyoDrum:EMGChannelCountMismatch', ...
            'Rest and active data must have one column per EMG channel.');
    end
    baseline = median(restVoltage, 1);
    noise = 1.4826 * median(abs(restVoltage - baseline), 1);
    active = localPercentile(activeVoltage, options.ActivePercentile);
    span = active - baseline;
    valid = isfinite(span) & span >= options.MinimumActiveSpanVolts;
    safeSpan = max(span, options.MinimumActiveSpanVolts);
    profile.baseline_V = baseline;
    profile.noiseSigma_V = noise;
    profile.activeReference_V = active;
    profile.activeSpan_V = safeSpan;
    profile.validChannel = valid;
    profile.hands = hands;
    profile.schemaVersion = "emg-confirmation-1";
end

function result = localPercentile(values, percentage)
    result = zeros(1, size(values, 2));
    for column = 1:size(values, 2)
        sorted = sort(values(:, column));
        position = 1 + (numel(sorted) - 1) * percentage / 100;
        low = floor(position);
        high = ceil(position);
        fraction = position - low;
        result(column) = sorted(low) * (1-fraction) + ...
            sorted(high) * fraction;
    end
end
