function block = prepareMLCollectionBlock(frames, plan, ...
        accelCalibration, gyroBiasDps, profile, cfg)
% PREPAREMLCOLLECTIONBLOCK Add calibrated signals and block labels to frames.

    arguments
        frames (1,1) struct
        plan table
        accelCalibration (1,2) cell
        gyroBiasDps (2,3) double {mustBeFinite}
        profile (1,1) struct
        cfg (1,1) struct
    end
    if height(plan) ~= 1
        error('MyoDrum:InvalidMLCollectionPlan', ...
            'Exactly one collection-plan row is required.');
    end
    required = {'sequence','timestamp_us','time_s','status','frameValid', ...
        'accelG','gyroDps','temperatureC','emgAdc'};
    if ~all(isfield(frames, required))
        error('MyoDrum:InvalidMLCollectionFrames', ...
            'The synchronized frame structure is incomplete.');
    end

    count = numel(frames.time_s);
    calibratedAccel = zeros(count, 3, 2);
    correctedGyro = zeros(count, 3, 2);
    for hand = 1:2
        calibratedAccel(:, :, hand) = applyMPU6050AccelCalibration( ...
            frames.accelG(:, :, hand), accelCalibration{hand});
        correctedGyro(:, :, hand) = deg2rad( ...
            frames.gyroDps(:, :, hand) - gyroBiasDps(hand, :));
    end
    volts = double(frames.emgAdc) * cfg.EMG.ADCReferenceVolts / ...
        cfg.EMG.ADCMaxCount;
    normalized = max(0, (volts - profile.baseline_V) ./ ...
        profile.activeSpan_V);

    block.time_s = frames.time_s;
    block.timestamp_us = frames.timestamp_us;
    block.sequenceNumber = frames.sequence;
    block.deviceStatus = frames.status;
    block.frameValid = frames.frameValid;
    block.accelRaw_g = frames.accelG;
    block.accelCalibrated_g = calibratedAccel;
    block.gyroRaw_dps = frames.gyroDps;
    block.gyroCorrected_rad_s = correctedGyro;
    block.temperature_C = frames.temperatureC;
    block.emgAdc = frames.emgAdc;
    block.emgVoltage_V = volts;
    block.emgNormalized = normalized;
    block.blockId = repmat(plan.BlockId, count, 1);
    block.round = repmat(plan.Round, count, 1);
    block.handIndex = repmat(plan.HandIndex, count, 1);
    block.blockClassIndex = repmat(plan.ClassIndex, count, 1);
end
