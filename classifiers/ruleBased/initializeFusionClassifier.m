function state = initializeFusionClassifier
% INITIALIZEFUSIONCLASSIFIER Create independent two-hand fusion state.
    pending = struct('active', false, 'candidateId', uint32(0), ...
        'candidateTime_s', NaN, 'expiresAt_s', NaN, 'label', "", ...
        'initialEMGScore', 0, 'peakEMGScore', 0);
    state.pending = repmat(pending, 1, 2);
    state.lastAcceptedTime_s = [-inf, -inf];
    state.nextCandidateId = uint32(1);
    state.historyTime_s = zeros(0, 1);
    state.historyScore = zeros(0, 2);
end
