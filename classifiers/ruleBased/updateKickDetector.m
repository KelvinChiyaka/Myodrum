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
            detector.peakDownstrokeRate = signedRate;
            detector.armedAtTime = timeNow;
        end
    elseif detector.mode == 2
        if (timeNow - detector.armedAtTime) > kickCfg.ArmTimeoutSeconds
            detector.mode = 1;
            detector.peakDownstrokeRate = 0;
            diagnostics.signedRateRadPerSec = signedRate;
            diagnostics.accelStepMps2 = max(detector.recentAccelSteps);
            diagnostics.state = detector.mode;
            return;
        end
        detector.peakDownstrokeRate = max( ...
            detector.peakDownstrokeRate, signedRate);
        reversed = signedRate <= kickCfg.BottomReversalRadPerSec;
        decelerated = (detector.peakDownstrokeRate - signedRate) >= ...
            kickCfg.DecelerationDropRadPerSec;
        jerked = max(detector.recentAccelSteps) >= ...
            kickCfg.AccelStepThresholdMps2;
        enoughTime = (timeNow - detector.lastHitTime) >= ...
            kickCfg.MinInterHitSeconds;
        if (reversed || decelerated) && jerked && enoughTime
            hit = true;
            detector.lastHitTime = timeNow;
            % Include the reversal sample itself. At the measured dual-I2C
            % rate it may be the only negative-rate sample before rebound.
            detector.returnTravelRad = max(0, -signedRate) * samplePeriod;
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
            if signedRate >= kickCfg.DownstrokeArmRadPerSec
                % A quick player may already be in the next downstroke by
                % the time MATLAB obtains the next frame. Arm it directly.
                detector.mode = 2;
                detector.recentAccelSteps(:) = 0;
                detector.peakDownstrokeRate = signedRate;
                detector.armedAtTime = timeNow;
            else
                detector.mode = 1;
            end
        end
    end

    detector.previousSignedRate = signedRate;
    diagnostics.signedRateRadPerSec = signedRate;
    diagnostics.accelStepMps2 = max(detector.recentAccelSteps);
    diagnostics.state = detector.mode;
end
