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
        options.ActiveSampleHands (:,1) string = strings(0,1)
        options.ActiveChannelMask (:,1) logical = false(0,1)
        options.RailVolts (1,1) double {mustBePositive} = inf
        options.MaximumRailFraction (1,1) double ...
            {mustBeInRange(options.MaximumRailFraction, 0, 1)} = 1
        options.MaximumRestSigmaVolts (1,1) double ...
            {mustBePositive} = inf
        options.MinimumSignalToNoiseRatio (1,1) double ...
            {mustBeNonnegative} = 0
        options.NoiseFloorVolts (1,1) double {mustBePositive} = 1e-6
    end
    channelCount = numel(hands);
    activeChannelMask = options.ActiveChannelMask;
    if isempty(activeChannelMask)
        activeChannelMask = true(channelCount,1);
    elseif numel(activeChannelMask) ~= channelCount
        error('MyoDrum:EMGActiveChannelMaskMismatch', ...
            'ActiveChannelMask must contain one value per EMG channel.');
    end
    if size(restVoltage, 2) ~= channelCount || ...
            size(activeVoltage, 2) ~= channelCount
        error('MyoDrum:EMGChannelCountMismatch', ...
            'Rest and active data must have one column per EMG channel.');
    end
    if ~isempty(options.ActiveSampleHands) && ...
            numel(options.ActiveSampleHands) ~= size(activeVoltage, 1)
        error('MyoDrum:EMGActiveHandCountMismatch', ...
            ['ActiveSampleHands must be empty or contain one hand label ' ...
             'for every row of activeVoltage.']);
    end
    baseline = median(restVoltage, 1);
    noise = 1.4826 * median(abs(restVoltage - baseline), 1);
    active = nan(1, channelCount);
    activeRailFraction = nan(1, channelCount);
    for channel = 1:channelCount
        if ~activeChannelMask(channel)
            active(channel) = baseline(channel);
            activeRailFraction(channel) = 0;
            continue;
        end
        selected = true(size(activeVoltage, 1), 1);
        if ~isempty(options.ActiveSampleHands)
            selected = options.ActiveSampleHands == hands(channel);
        end
        if ~any(selected)
            error('MyoDrum:MissingEMGActiveHandSamples', ...
                'No active calibration samples were supplied for %s.', ...
                hands(channel));
        end
        values = activeVoltage(selected, channel);
        active(channel) = localPercentile(values, ...
            options.ActivePercentile);
        activeRailFraction(channel) = mean(values >= options.RailVolts);
    end
    span = active - baseline;
    restRailFraction = mean(restVoltage >= options.RailVolts, 1);
    signalToNoise = span ./ max(noise, options.NoiseFloorVolts);
    hasActiveRise = isfinite(span) & ...
        span >= options.MinimumActiveSpanVolts;
    restIsQuiet = noise <= options.MaximumRestSigmaVolts;
    isNotRailed = active < options.RailVolts & ...
        restRailFraction <= options.MaximumRailFraction & ...
        activeRailFraction <= options.MaximumRailFraction;
    hasSignalToNoise = signalToNoise >= ...
        options.MinimumSignalToNoiseRatio;
    valid = hasActiveRise & restIsQuiet & isNotRailed & ...
        hasSignalToNoise & activeChannelMask(:).';
    safeSpan = max(span, options.MinimumActiveSpanVolts);
    reason = repmat("OK", 1, channelCount);
    for channel = 1:channelCount
        if ~activeChannelMask(channel)
            reason(channel) = "INACTIVE";
            continue;
        end
        failures = strings(0,1);
        if ~hasActiveRise(channel)
            failures(end+1,1) = "NO_ACTIVE_RISE"; %#ok<AGROW>
        end
        if ~restIsQuiet(channel)
            failures(end+1,1) = "NOISY_REST"; %#ok<AGROW>
        end
        if ~isNotRailed(channel)
            failures(end+1,1) = "RAILED_OR_SATURATED"; %#ok<AGROW>
        end
        if ~hasSignalToNoise(channel)
            failures(end+1,1) = "LOW_SIGNAL_TO_NOISE"; %#ok<AGROW>
        end
        if ~isempty(failures)
            reason(channel) = strjoin(failures, "+");
        end
    end
    profile.baseline_V = baseline;
    profile.noiseSigma_V = noise;
    profile.activeReference_V = active;
    profile.activeSpan_V = safeSpan;
    profile.signalToNoiseRatio = signalToNoise;
    profile.restRailFraction = restRailFraction;
    profile.activeRailFraction = activeRailFraction;
    profile.validChannel = valid;
    profile.qualityReason = reason;
    profile.hands = hands;
    profile.schemaVersion = "emg-confirmation-2";
end

function result = localPercentile(values, percentage)
    if isvector(values)
        values = values(:);
    end
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
