function log = initializeIMUEMGLiveLog(capacity)
% INITIALIZEIMUEMGLIVELOG Preallocate synchronized live-session signals.

    arguments
        capacity (1,1) double {mustBeInteger, mustBePositive}
    end
    log.sampleCount = 0;
    log.capacity = capacity;
    log.time_s=zeros(capacity,1); log.sequenceNumber=zeros(capacity,1,'uint16');
    log.deviceStatus=zeros(capacity,1,'uint8'); log.frameValid=false(capacity,2);
    log.accelRaw_g=zeros(capacity,3,2); log.accelCalibrated_g=zeros(capacity,3,2);
    log.gyroCorrected_rad_s=zeros(capacity,3,2); log.temperature_C=zeros(capacity,2);
    log.yawPitchRoll_deg=zeros(capacity,3,2); log.emgVoltage_V=zeros(capacity,4);
    log.emgNormalized=zeros(capacity,4); log.imuCandidateHit=false(capacity,2);
    log.emgConfirmedHit=false(capacity,2); log.fusionAcceptedHit=false(capacity,2);
    log.fusionRejectedHit=false(capacity,2); log.fusionPendingActive=false(capacity,2);
    log.emgConfirmationScore=zeros(capacity,2);
    log.emgRuntimeValid=false(capacity,4);
    log.fusionDecisionCode=zeros(capacity,2,'uint8');
    log.candidateTargetIndex=zeros(capacity,2,'uint8');
    log.targetIndex=zeros(capacity,2,'uint8');
    log.hostProcessingLag_s=nan(capacity,1);
end
