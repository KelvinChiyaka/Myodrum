function quality = summarizeEMGQuality(emgVoltageV, restBaseline, qualityCfg)
% SUMMARIZEEMGQUALITY Calculate per-channel range and signal-health flags.

    arguments
        emgVoltageV (:,:) double
        restBaseline
        qualityCfg (1,1) struct
    end
    if isempty(emgVoltageV)
        quality = struct;
        return;
    end

    quality.minimum_V = min(emgVoltageV, [], 1, 'omitnan');
    quality.median_V = median(emgVoltageV, 1, 'omitnan');
    quality.maximum_V = max(emgVoltageV, [], 1, 'omitnan');
    quality.range_V = quality.maximum_V - quality.minimum_V;
    quality.percentile95_V = columnPercentile(emgVoltageV, 95);
    quality.neverResponded = quality.range_V < qualityCfg.FlatRangeVolts;
    quality.railed = quality.maximum_V >= qualityCfg.RailVolts;
    if isempty(restBaseline)
        quality.noisyRest = false(size(quality.minimum_V));
    else
        quality.noisyRest = ...
            restBaseline.robustSigma_V > qualityCfg.RestSigmaWarnVolts;
    end
    quality.hasNonfinite = any(~isfinite(emgVoltageV), 1);
end

function values = columnPercentile(data, percentage)
    values = nan(1, size(data, 2));
    for columnIndex = 1:size(data, 2)
        column = sort(data(isfinite(data(:, columnIndex)), columnIndex));
        if isempty(column)
            continue;
        end
        position = 1 + (numel(column) - 1) * percentage / 100;
        lowerIndex = floor(position);
        upperIndex = ceil(position);
        fraction = position - lowerIndex;
        values(columnIndex) = column(lowerIndex) * (1 - fraction) + ...
            column(upperIndex) * fraction;
    end
end
