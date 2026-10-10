function baseline = computeRestBaseline(emgVoltageV)
% COMPUTERESTBASELINE Return robust per-channel resting EMG statistics.

    arguments
        emgVoltageV (:,:) double
    end
    if isempty(emgVoltageV)
        baseline = [];
        return;
    end
    baseline.median_V = median(emgVoltageV, 1, 'omitnan');
    deviations = abs(emgVoltageV - baseline.median_V);
    baseline.robustSigma_V = 1.4826 * median(deviations, 1, 'omitnan');
end
