function correctedAccelG = applyMPU6050AccelCalibration(rawAccelG, calibration)
% APPLYMPU6050ACCELCALIBRATION Apply six-pose offset and scale correction.

    arguments
        rawAccelG (:,3) double {mustBeFinite}
        calibration (1,1) struct
    end

    required = {'accelOffsetG', 'accelScale'};
    for k = 1:numel(required)
        if ~isfield(calibration, required{k}) || ...
                ~isequal(size(calibration.(required{k})), [1, 3])
            error('MyoDrum:InvalidAccelCalibration', ...
                'Calibration must contain 1x3 %s.', required{k});
        end
    end
    if any(~isfinite(calibration.accelScale)) || ...
            any(abs(calibration.accelScale) < 1e-6)
        error('MyoDrum:InvalidAccelScale', ...
            'Accelerometer scale values must be finite and nonzero.');
    end

    correctedAccelG = (rawAccelG - calibration.accelOffsetG) ./ ...
        calibration.accelScale;
end
