function [detector, hit, diagnostics] = updateKickDetector( ...
        detector, accelMps2, gyroRadPerSec, direction, timeNow, ...
        sampleRateHz, kickCfg)
% UPDATEKICKDETECTOR Detect one robust downstroke/bottom-reversal event.

    signedRate = direction.sign * gyroRadPerSec(direction.axis);
    if isempty(detector.previousAccelMps2)
        accelStep = 0;
    else
        accelStep = norm(accelMps2 - detector.previousAccelMps2);
    end
    detector.previousAccelMps2 = accelMps2;
    detector.recentAccelSteps = [detector.recentAccelSteps(2:3), accelStep];

    if isnan(detector.lastSampleTime)
        samplePeriod = 1 / sampleRateHz;
    else
        samplePeriod = min(max(timeNow - detector.lastSampleTime, 0), ...
            2 / sampleRateHz);
    end
    detector.lastSampleTime = timeNow;
    hit = false;

    if detector.mode == 1
        if signedRate >= kickCfg.DownstrokeArmRadPerSec
            detector.mode = 2;
            detector.recentAccelSteps(:) = 0;
        end
    elseif detector.mode == 2
        reversed = signedRate <= kickCfg.BottomReversalRadPerSec;
        jerked = max(detector.recentAccelSteps) >= ...
            kickCfg.AccelStepThresholdMps2;
        enoughTime = (timeNow - detector.lastHitTime) >= ...
            kickCfg.MinInterHitSeconds;
        if reversed && jerked && enoughTime
            hit = true;
            detector.lastHitTime = timeNow;
            detector.returnTravelRad = 0;
            detector.mode = 3;
        elseif signedRate <= kickCfg.MissedHitResetRadPerSec
            detector.mode = 1;
        end
    else
        detector.returnTravelRad = detector.returnTravelRad + ...
            max(0, -signedRate) * samplePeriod;
        enoughTravel = detector.returnTravelRad >= ...
            kickCfg.MinReturnTravelRad;
        enoughTime = (timeNow - detector.lastHitTime) >= ...
            kickCfg.MinInterHitSeconds;
        if enoughTravel && enoughTime
            detector.mode = 1;
        end
    end

    diagnostics.signedRateRadPerSec = signedRate;
    diagnostics.accelStepMps2 = max(detector.recentAccelSteps);
    diagnostics.state = detector.mode;
end
