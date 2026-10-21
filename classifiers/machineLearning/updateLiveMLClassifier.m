function [state, decision, events] = updateLiveMLClassifier( ...
        state, time_s, accel_g, gyro_rad_s, normalizedEMG, ...
        frameValid, emgValid, imuCandidate, cfg, ruleLabels)
% UPDATELIVEMLCLASSIFIER Resolve IMU candidates with ML and an EMG gate.

    arguments
        state (1,1) struct
        time_s (1,1) double {mustBeFinite}
        accel_g (1,3,2) double {mustBeFinite}
        gyro_rad_s (1,3,2) double {mustBeFinite}
        normalizedEMG (1,:) double {mustBeFinite, mustBeNonnegative}
        frameValid (1,2) logical
        emgValid (1,:) logical
        imuCandidate (1,2) logical
        cfg (1,1) struct
        ruleLabels (1,2) string = strings(1,2)
    end

    state = appendSample(state, time_s, accel_g, gyro_rad_s, ...
        normalizedEMG, frameValid, emgValid);
    decision = emptyDecision(state.pendingActive);
    events = emptyEvents;

    % Resolve an older candidate only after its post-strike samples exist.
    for hand = 1:2
        if ~state.pendingActive(hand) || time_s < state.pendingTime_s(hand) + ...
                cfg.MachineLearning.FeatureWindowPostSeconds
            continue;
        end
        [state, result, event] = resolveCandidate(state, hand, time_s, cfg);
        decision = mergeDecision(decision, result, hand);
        events(end+1) = event; %#ok<AGROW>
    end

    % Admit new candidates after resolving old ones so the two hands remain
    % independent and simultaneous strikes can both become pending.
    for hand = 1:2
        if ~imuCandidate(hand), continue; end
        if state.pendingActive(hand)
            event = makeEvent(state.nextCandidateId, hand, time_s, time_s, ...
                "", ruleLabels(hand), 0, 0, false, ...
                "PENDING_ALREADY_ACTIVE");
            state.nextCandidateId = state.nextCandidateId + 1;
            decision.rejected(hand) = true;
            decision.reason(hand) = "PENDING_ALREADY_ACTIVE";
            events(end+1) = event; %#ok<AGROW>
        else
            state.pendingActive(hand) = true;
            state.pendingTime_s(hand) = time_s;
            state.pendingId(hand) = state.nextCandidateId;
            state.pendingRuleLabel(hand) = ruleLabels(hand);
            state.nextCandidateId = state.nextCandidateId + 1;
        end
    end
    decision.pendingActive = state.pendingActive;
end

function state = appendSample(state, time_s, accel, gyro, emg, valid, emgValid)
    index = mod(state.writeIndex, state.capacity) + 1;
    state.writeIndex = index;
    state.sampleCount = min(state.sampleCount + 1, state.capacity);
    state.time_s(index) = time_s;
    state.accel_g(index,:,:) = accel;
    state.gyro_rad_s(index,:,:) = gyro;
    state.emgNormalized(index,:) = emg;
    state.frameValid(index,:) = valid;
    state.emgValid(index,:) = emgValid;
end

function [state, decision, event] = resolveCandidate(state, hand, now_s, cfg)
    candidateTime = state.pendingTime_s(hand);
    ruleLabel = state.pendingRuleLabel(hand);
    startTime = candidateTime - ...
        cfg.MachineLearning.FeatureWindowPreSeconds;
    endTime = candidateTime + ...
        cfg.MachineLearning.FeatureWindowPostSeconds;
    rows = find(state.time_s >= startTime & state.time_s <= endTime);
    [~, order] = sort(state.time_s(rows));
    rows = rows(order);
    expected = (cfg.MachineLearning.FeatureWindowPreSeconds + ...
        cfg.MachineLearning.FeatureWindowPostSeconds) * ...
        cfg.EMG.CombinedStreamRateHz;
    minimumRows = floor(expected * ...
        cfg.MachineLearning.LiveMinimumWindowFraction);
    handChannels = find(cfg.EMG.ActiveChannelMask(:) & ...
        cfg.EMG.Hands == cfg.IMU.HandNames(hand));

    label = "";
    confidence = 0;
    emgScore = 0;
    accepted = false;
    reason = "INVALID_WINDOW";
    % A single missed IMU frame must not invalidate an otherwise complete
    % 300 ms feature window. Remove invalid motion rows and proceed when the
    % configured minimum amount of real data remains.
    validRows = rows(state.frameValid(rows,hand));
    if numel(validRows) >= minimumRows
        rows = validRows;
        emgWindow = state.emgNormalized(rows,:);
        requiredChannels = cfg.Fusion.RequiredActiveChannels;
        emgHealthy = numel(handChannels) >= requiredChannels && ...
            all(all(state.emgValid(rows,handChannels)));
        if numel(handChannels) >= requiredChannels
            emgPeaks = max(emgWindow(:,handChannels),[],1);
            emgPeaks = sort(emgPeaks, 'descend');
            emgScore = emgPeaks(requiredChannels);
        end
        if ~emgHealthy
            reason = "EMG_INVALID";
        else
            accel = reshape(state.accel_g(rows,:,hand), [], 3);
            gyro = reshape(state.gyro_rad_s(rows,:,hand), [], 3);
            [values, names] = computeMLWindowFeatures(accel, gyro, ...
                emgWindow, state.time_s(rows), hand, cfg);
            if ~isequal(names(:), state.predictorNames)
                error('MyoDrum:LiveMLPredictorMismatch', ...
                    'Live feature order differs from the trained model.');
            end
            predictor = array2table(values, ...
                'VariableNames', cellstr(state.predictorNames));
            [prediction, scores] = predict(state.classifier, predictor);
            label = string(prediction);
            modelClasses = string(state.classifier.ClassNames(:));
            classIndex = find(modelClasses == label, 1);
            if isempty(classIndex)
                error('MyoDrum:LiveMLUnknownPrediction', ...
                    'The live model returned an unknown class.');
            end
            confidence = scores(classIndex);
            if label == "NO_HIT"
                reason = "ML_NO_HIT";
            elseif ~ismember(label, cfg.Targets.Names)
                reason = "ML_UNKNOWN_CLASS";
            elseif confidence < cfg.MachineLearning.LiveMinimumConfidence
                reason = "ML_LOW_CONFIDENCE";
            elseif cfg.MachineLearning.RequireEMGConfirmation && ...
                    emgScore < cfg.Fusion.EMGThresholdNormalized
                reason = "EMG_BELOW_THRESHOLD";
            elseif state.classifierMode == "hybrid" && ruleLabel == ""
                reason = "HYBRID_RULE_UNCERTAIN";
            elseif state.classifierMode == "hybrid" && ruleLabel ~= label
                reason = "HYBRID_DISAGREEMENT";
            elseif now_s-state.lastAcceptedTime_s(hand) < ...
                    cfg.Fusion.MinimumInterAcceptedHitSeconds
                reason = "DUPLICATE_GUARD";
            else
                accepted = true;
                reason = "ACCEPTED";
                state.lastAcceptedTime_s(hand) = now_s;
            end
        end
    end

    id = state.pendingId(hand);
    event = makeEvent(id, hand, candidateTime, now_s, label, ruleLabel, ...
        confidence, emgScore, accepted, reason);
    decision = emptyDecision(false(1,2));
    decision.accepted(hand) = accepted;
    decision.rejected(hand) = ~accepted;
    decision.labels(hand) = label;
    decision.ruleLabels(hand) = ruleLabel;
    decision.reason(hand) = reason;
    decision.emgScore(hand) = emgScore;
    decision.mlConfidence(hand) = confidence;
    state.pendingActive(hand) = false;
    state.pendingTime_s(hand) = NaN;
    state.pendingId(hand) = uint32(0);
    state.pendingRuleLabel(hand) = "";
end

function combined = mergeDecision(combined, addition, hand)
    combined.accepted(hand) = addition.accepted(hand);
    combined.rejected(hand) = addition.rejected(hand);
    combined.labels(hand) = addition.labels(hand);
    combined.ruleLabels(hand) = addition.ruleLabels(hand);
    combined.reason(hand) = addition.reason(hand);
    combined.emgScore(hand) = addition.emgScore(hand);
    combined.mlConfidence(hand) = addition.mlConfidence(hand);
end

function decision = emptyDecision(pending)
    decision.accepted = false(1,2);
    decision.rejected = false(1,2);
    decision.labels = strings(1,2);
    decision.ruleLabels = strings(1,2);
    decision.reason = strings(1,2);
    decision.emgScore = zeros(1,2);
    decision.mlConfidence = zeros(1,2);
    decision.pendingActive = logical(pending);
end

function event = makeEvent(id, hand, candidateTime, decisionTime, label, ...
        ruleLabel, confidence, emgScore, accepted, reason)
    event.candidateId = uint32(id);
    event.handIndex = uint8(hand);
    event.candidateTime_s = candidateTime;
    event.decisionTime_s = decisionTime;
    event.decisionLatency_s = decisionTime-candidateTime;
    event.orientationLabel = label;
    event.mlLabel = label;
    event.ruleLabel = ruleLabel;
    event.initialEMGScore = emgScore;
    event.peakEMGScore = emgScore;
    event.mlConfidence = confidence;
    event.accepted = logical(accepted);
    event.reason = reason;
end

function events = emptyEvents
    template = makeEvent(0,0,NaN,NaN,"","",0,0,false,"");
    events = template([]);
end
