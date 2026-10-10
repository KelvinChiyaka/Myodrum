function calibration = loadOrRunMPU6050AccelCalibration( ...
        readSample, sampleRate, calibrationFile, forceCalibration, poseSeconds)
% LOADORRUNMPU6050ACCELCALIBRATION Load or perform six-pose calibration.
%
%   calibration = loadOrRunMPU6050AccelCalibration(readSample, sampleRate, ...
%       calibrationFile, forceCalibration, poseSeconds)
%
% readSample must be a function handle returning a struct with accelG and
% gyroDps row vectors. The six-pose result corrects accelerometer offset and
% per-axis scale. A fresh stationary gyro bias should still be measured at the
% beginning of every playing session because gyro bias changes with temperature.

    arguments
        readSample (1,1) function_handle
        sampleRate (1,1) double {mustBeFinite, mustBePositive}
        calibrationFile (1,1) string
        forceCalibration (1,1) logical = false
        poseSeconds (1,1) double {mustBeFinite, mustBePositive} = 2.0
    end

    if ~forceCalibration && isfile(calibrationFile)
        saved = load(calibrationFile, 'calibration');
        if isfield(saved, 'calibration') && isValidCalibration(saved.calibration)
            calibration = saved.calibration;
            fprintf('Loaded six-pose accelerometer calibration:\n  %s\n', ...
                calibrationFile);
            printCalibration(calibration);
            return;
        end
        warning('MyoDrum:InvalidSavedCalibration', ...
            'Saved calibration is invalid and will be replaced: %s', ...
            calibrationFile);
    end

    calibration = performSixPose(readSample, sampleRate, poseSeconds);
    outputFolder = fileparts(calibrationFile);
    if ~isempty(outputFolder) && ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    save(calibrationFile, 'calibration');
    fprintf('Saved six-pose accelerometer calibration:\n  %s\n', ...
        calibrationFile);
    printCalibration(calibration);
end

function calibration = performSixPose(readSample, sampleRate, poseSeconds)
    poseNames = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
    poseAxes = [1, 1, 2, 2, 3, 3];
    poseSigns = [1, -1, 1, -1, 1, -1];
    sampleCount = max(20, round(sampleRate * poseSeconds));

    maxGyroNoiseDps = 5.0;
    gravityMagnitudeRange = [0.70, 1.30];
    minimumNamedAxisG = 0.55;
    poseMean = zeros(6, 3);
    poseStd = zeros(6, 3);

    fprintf('\nSIX-POSE ACCELEROMETER CALIBRATION\n');
    fprintf(['This is required once for this sensor mounting. The result is ' ...
        'saved and reused on later runs.\n']);
    fprintf('For each pose, hold the complete stick still with the named SENSOR axis up.\n');
    fprintf('Each accepted measurement takes approximately %.1f seconds.\n\n', ...
        poseSeconds);

    for poseIndex = 1:6
        accepted = false;
        while ~accepted
            input(sprintf('Pose %d/6: %s upward. Hold still and press Enter...', ...
                poseIndex, poseNames(poseIndex)), 's');

            % Discard a few frames disturbed while the operator released Enter.
            for k = 1:3
                readSample();
            end

            accel = zeros(sampleCount, 3);
            gyro = zeros(sampleCount, 3);
            for k = 1:sampleCount
                sample = readSample();
                validateSample(sample);
                accel(k, :) = sample.accelG;
                gyro(k, :) = sample.gyroDps;
            end

            meanAccel = median(accel, 1);
            accelNoise = 1.4826 * median(abs(accel - meanAccel), 1);
            medianGyro = median(gyro, 1);
            gyroNoise = 1.4826 * median(abs(gyro - medianGyro), 1);
            gravityMagnitude = norm(meanAccel);
            namedAxisValue = poseSigns(poseIndex) * ...
                meanAccel(poseAxes(poseIndex));
            otherAxes = setdiff(1:3, poseAxes(poseIndex));
            namedAxisIsDominant = namedAxisValue > ...
                max(abs(meanAccel(otherAxes)));

            fprintf('  Mean accel: [%+.4f %+.4f %+.4f] g; |a| = %.4f g\n', ...
                meanAccel, gravityMagnitude);

            accepted = all(isfinite([meanAccel, gyroNoise])) && ...
                all(gyroNoise <= maxGyroNoiseDps) && ...
                gravityMagnitude >= gravityMagnitudeRange(1) && ...
                gravityMagnitude <= gravityMagnitudeRange(2) && ...
                namedAxisValue >= minimumNamedAxisG && namedAxisIsDominant;

            if ~accepted
                fprintf(['  Pose rejected. Check the named sensor axis, hold the ' ...
                    'stick still, and retry.\n']);
                fprintf('  Gyro noise: [%.3f %.3f %.3f] deg/s\n\n', gyroNoise);
            else
                poseMean(poseIndex, :) = meanAccel;
                poseStd(poseIndex, :) = accelNoise;
                fprintf('  Pose accepted.\n\n');
            end
        end
    end

    positive = poseMean([1, 3, 5], :);
    negative = poseMean([2, 4, 6], :);
    axisIndex = 1:3;
    positiveAxis = positive(sub2ind([3, 3], axisIndex, axisIndex));
    negativeAxis = negative(sub2ind([3, 3], axisIndex, axisIndex));

    calibration.accelOffsetG = (positiveAxis + negativeAxis) / 2;
    calibration.accelScale = (positiveAxis - negativeAxis) / 2;
    if any(~isfinite(calibration.accelScale)) || ...
            any(calibration.accelScale < 0.65) || ...
            any(calibration.accelScale > 1.35)
        error('MyoDrum:ImplausibleAccelCalibration', ...
            ['Six-pose scale is implausible: [%.3f %.3f %.3f]. ' ...
             'Repeat the calibration and verify every sensor-axis direction.'], ...
            calibration.accelScale);
    end

    correctedPose = (poseMean - calibration.accelOffsetG) ./ ...
        calibration.accelScale;
    expected = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    residual = correctedPose - expected;

    calibration.poseNames = poseNames;
    calibration.poseMeanAccelG = poseMean;
    calibration.poseNoiseG = poseStd;
    calibration.correctedPoseAccelG = correctedPose;
    calibration.maxPoseResidualG = max(abs(residual), [], 'all');
    calibration.sampleRateHz = sampleRate;
    calibration.samplesPerPose = sampleCount;
    calibration.createdAt = string(datetime('now', 'TimeZone', 'local'));
    calibration.schemaVersion = "1.0";

    if calibration.maxPoseResidualG > 0.25
        error('MyoDrum:PoorAccelCalibrationFit', ...
            ['Six-pose residual is %.3f g, above the 0.25 g limit. ' ...
             'Repeat the calibration with more accurate poses.'], ...
            calibration.maxPoseResidualG);
    end
end

function validateSample(sample)
    required = {'accelG', 'gyroDps'};
    for k = 1:numel(required)
        if ~isfield(sample, required{k}) || ...
                ~isequal(size(sample.(required{k})), [1, 3]) || ...
                any(~isfinite(sample.(required{k})))
            error('MyoDrum:InvalidIMUSample', ...
                'The IMU returned an invalid %s sample.', required{k});
        end
    end
end

function valid = isValidCalibration(calibration)
    valid = isstruct(calibration) && ...
        isfield(calibration, 'accelOffsetG') && ...
        isfield(calibration, 'accelScale') && ...
        isequal(size(calibration.accelOffsetG), [1, 3]) && ...
        isequal(size(calibration.accelScale), [1, 3]) && ...
        all(isfinite(calibration.accelOffsetG)) && ...
        all(isfinite(calibration.accelScale)) && ...
        all(calibration.accelScale > 0.65) && ...
        all(calibration.accelScale < 1.35);
end

function printCalibration(calibration)
    fprintf('  Accel offset [X Y Z]: %+.4f %+.4f %+.4f g\n', ...
        calibration.accelOffsetG);
    fprintf('  Accel scale  [X Y Z]:  %.4f  %.4f  %.4f\n', ...
        calibration.accelScale);
    if isfield(calibration, 'maxPoseResidualG')
        fprintf('  Maximum six-pose residual: %.4f g\n', ...
            calibration.maxPoseResidualG);
    end
end
