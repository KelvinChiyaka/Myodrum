function result = findCoupledEMGChannels(values, channelNames, options)
% FINDCOUPLEDEMGCHANNELS Flag suspiciously similar active EMG channels.

    arguments
        values (:,:) double
        channelNames (:,1) string
        options.MaximumCorrelation (1,1) double = 0.98
        options.MinimumRangeVolts (1,1) double = 0.05
    end
    if size(values, 2) ~= numel(channelNames)
        error('MyoDrum:EMGChannelCountMismatch', ...
            'The data columns must match the configured channel names.');
    end
    pairs = zeros(0, 2);
    correlations = zeros(0, 1);
    ranges = max(values, [], 1) - min(values, [], 1);
    matrix = corr(values, 'Rows', 'complete');
    for first = 1:size(values, 2)
        for second = first + 1:size(values, 2)
            coefficient = matrix(first, second);
            if ranges(first) >= options.MinimumRangeVolts && ...
                    ranges(second) >= options.MinimumRangeVolts && ...
                    isfinite(coefficient) && ...
                    abs(coefficient) >= options.MaximumCorrelation
                pairs(end + 1, :) = [first, second]; %#ok<AGROW>
                correlations(end + 1, 1) = coefficient; %#ok<AGROW>
            end
        end
    end
    result.pairs = pairs;
    result.correlations = correlations;
    result.correlationMatrix = matrix;
    result.channelNames = channelNames;
end
