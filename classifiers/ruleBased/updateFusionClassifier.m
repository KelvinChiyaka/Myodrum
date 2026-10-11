function [state, decision, events] = updateFusionClassifier( ...
        state, time_s, imuCandidate, orientationLabels, normalizedEMG, ...
        channelHands, validChannels, fusionCfg)
% UPDATEFUSIONCLASSIFIER Confirm IMU-labelled candidates using EMG activity.

    arguments
        state (1,1) struct
        time_s (1,1) double {mustBeFinite}
        imuCandidate (1,2) logical
        orientationLabels (1,2) string
        normalizedEMG (1,:) double {mustBeFinite, mustBeNonnegative}
        channelHands (:,1) string
        validChannels (:,1) logical
        fusionCfg (1,1) struct
    end
    if numel(normalizedEMG) ~= numel(channelHands) || ...
            numel(validChannels) ~= numel(channelHands)
        error('MyoDrum:FusionChannelCountMismatch', ...
            'EMG values, channel hands and validity must have equal lengths.');
    end
    handNames = ["LEFT", "RIGHT"];
    currentScore = zeros(1, 2);
    for hand = 1:2
        eligible = channelHands == handNames(hand) & validChannels;
        values = sort(normalizedEMG(eligible), 'descend');
        required = fusionCfg.RequiredActiveChannels;
        if numel(values) >= required
            currentScore(hand) = values(required);
        end
    end
    state.historyTime_s(end+1,1) = time_s;
    state.historyScore(end+1,:) = currentScore;
    keep = state.historyTime_s >= time_s - fusionCfg.EMGLookbackSeconds;
    state.historyTime_s = state.historyTime_s(keep);
    state.historyScore = state.historyScore(keep,:);

    decision.accepted = false(1,2);
    decision.rejected = false(1,2);
    decision.labels = strings(1,2);
    decision.reason = strings(1,2);
    decision.emgScore = currentScore;
    decision.pendingActive = false(1,2);
    events = emptyEvents;

    % Resolve older pending candidates before admitting new ones. The IMU
    % duplicate guard normally prevents both operations for one hand/frame.
    for hand = 1:2
        if ~state.pending(hand).active, continue; end
        state.pending(hand).peakEMGScore = max( ...
            state.pending(hand).peakEMGScore, currentScore(hand));
        if state.pending(hand).peakEMGScore >= ...
                fusionCfg.EMGThresholdNormalized
            [state, decision, events] = resolvePending(state, decision, ...
                events, hand, time_s, true, "ACCEPTED");
        elseif time_s >= state.pending(hand).expiresAt_s
            [state, decision, events] = resolvePending(state, decision, ...
                events, hand, time_s, false, "EMG_TIMEOUT");
        end
    end

    for hand = 1:2
        if ~imuCandidate(hand), continue; end
        label = orientationLabels(hand);
        lookbackScore = max(state.historyScore(:,hand));
        if label == ""
            events(end+1) = makeEvent(state.nextCandidateId, hand, ...
                time_s, time_s, label, lookbackScore, lookbackScore, ...
                false, "UNCERTAIN_ORIENTATION"); %#ok<AGROW>
            state.nextCandidateId = state.nextCandidateId + 1;
            decision.rejected(hand) = true;
            decision.reason(hand) = "UNCERTAIN_ORIENTATION";
        elseif time_s - state.lastAcceptedTime_s(hand) < ...
                fusionCfg.MinimumInterAcceptedHitSeconds
            events(end+1) = makeEvent(state.nextCandidateId, hand, ...
                time_s, time_s, label, lookbackScore, lookbackScore, ...
                false, "DUPLICATE_GUARD"); %#ok<AGROW>
            state.nextCandidateId = state.nextCandidateId + 1;
            decision.rejected(hand) = true;
            decision.reason(hand) = "DUPLICATE_GUARD";
        elseif state.pending(hand).active
            events(end+1) = makeEvent(state.nextCandidateId, hand, ...
                time_s, time_s, label, lookbackScore, lookbackScore, ...
                false, "PENDING_ALREADY_ACTIVE"); %#ok<AGROW>
            state.nextCandidateId = state.nextCandidateId + 1;
            decision.rejected(hand) = true;
            decision.reason(hand) = "PENDING_ALREADY_ACTIVE";
        else
            state.pending(hand).active = true;
            state.pending(hand).candidateId = state.nextCandidateId;
            state.pending(hand).candidateTime_s = time_s;
            state.pending(hand).expiresAt_s = time_s + ...
                fusionCfg.EMGConfirmationTimeoutSeconds;
            state.pending(hand).label = label;
            state.pending(hand).initialEMGScore = lookbackScore;
            state.pending(hand).peakEMGScore = lookbackScore;
            state.nextCandidateId = state.nextCandidateId + 1;
            if lookbackScore >= fusionCfg.EMGThresholdNormalized
                [state, decision, events] = resolvePending(state, decision, ...
                    events, hand, time_s, true, "ACCEPTED");
            end
        end
    end
    for hand = 1:2
        decision.pendingActive(hand) = state.pending(hand).active;
        if state.pending(hand).active
            decision.emgScore(hand) = state.pending(hand).peakEMGScore;
        end
    end
end

function [state, decision, events] = resolvePending( ...
        state, decision, events, hand, decisionTime, accepted, reason)
    pending = state.pending(hand);
    events(end+1) = makeEvent(pending.candidateId, hand, ...
        pending.candidateTime_s, decisionTime, pending.label, ...
        pending.initialEMGScore, pending.peakEMGScore, accepted, reason);
    decision.accepted(hand) = accepted;
    decision.rejected(hand) = ~accepted;
    decision.labels(hand) = pending.label;
    decision.reason(hand) = reason;
    decision.emgScore(hand) = pending.peakEMGScore;
    if accepted
        state.lastAcceptedTime_s(hand) = decisionTime;
    end
    state.pending(hand).active = false;
end

function event = makeEvent(id, hand, candidateTime, decisionTime, label, ...
        initialScore, peakScore, accepted, reason)
    event.candidateId = uint32(id);
    event.handIndex = uint8(hand);
    event.candidateTime_s = candidateTime;
    event.decisionTime_s = decisionTime;
    event.decisionLatency_s = decisionTime - candidateTime;
    event.orientationLabel = label;
    event.initialEMGScore = initialScore;
    event.peakEMGScore = peakScore;
    event.accepted = logical(accepted);
    event.reason = reason;
end

function events = emptyEvents
    template = makeEvent(uint32(0), 0, NaN, NaN, "", 0, 0, false, "");
    events = template([]);
end
