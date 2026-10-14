function log = appendIMUEMGLiveLog(log, frames, row, raw, calibrated, ...
        gyro, ypr, volts, normalizedEMG, candidate, decision, ...
        candidateTarget, target, runtimeValid, hostProcessingLag)
% APPENDIMUEMGLIVELOG Store one processed frame without per-row growth.
%
% This function is in the 200 Hz path. Its inputs have already been checked
% at the serial, calibration and classifier boundaries, so an arguments block
% is intentionally avoided here.

    next = log.sampleCount + 1;
    if next > log.capacity
        log = growLog(log, max(next, 2 * log.capacity));
    end
    log.time_s(next,1)=frames.time_s(row);
    log.sequenceNumber(next,1)=frames.sequence(row);
    log.deviceStatus(next,1)=frames.status(row);
    log.frameValid(next,:)=frames.frameValid(row,:);
    log.accelRaw_g(next,:,:)=raw;
    log.accelCalibrated_g(next,:,:)=calibrated;
    log.gyroCorrected_rad_s(next,:,:)=gyro;
    log.temperature_C(next,:)=frames.temperatureC(row,:);
    log.yawPitchRoll_deg(next,:,:)=ypr;
    log.emgVoltage_V(next,:)=volts;
    log.emgNormalized(next,:)=normalizedEMG;
    log.imuCandidateHit(next,:)=candidate;
    log.emgConfirmedHit(next,:)=decision.accepted;
    log.fusionAcceptedHit(next,:)=decision.accepted;
    log.fusionRejectedHit(next,:)=decision.rejected;
    log.fusionPendingActive(next,:)=decision.pendingActive;
    log.emgConfirmationScore(next,:)=decision.emgScore;
    log.emgRuntimeValid(next,:)=runtimeValid;
    log.fusionDecisionCode(next,:)=encodeDecision(decision);
    log.candidateTargetIndex(next,:)=candidateTarget;
    log.targetIndex(next,:)=target;
    log.hostProcessingLag_s(next,1)=hostProcessingLag;
    log.sampleCount = next;
end

function log = growLog(log, newCapacity)
    names = setdiff(fieldnames(log), {'sampleCount','capacity'}, 'stable');
    for fieldIndex = 1:numel(names)
        name = names{fieldIndex};
        value = log.(name);
        indices = repmat({':'}, 1, ndims(value));
        indices{1} = newCapacity;
        for dimension = 2:ndims(value)
            indices{dimension} = size(value, dimension);
        end
        value(indices{:}) = zeros(1, 'like', value);
        log.(name) = value;
    end
    log.capacity = newCapacity;
end

function codes = encodeDecision(decision)
    codes = zeros(1,2,'uint8');
    reasons = ["ACCEPTED", "EMG_TIMEOUT", "UNCERTAIN_ORIENTATION", ...
        "DUPLICATE_GUARD", "PENDING_ALREADY_ACTIVE"];
    for hand = 1:2
        match = find(reasons == decision.reason(hand), 1);
        if ~isempty(match), codes(hand) = uint8(match); end
    end
end
