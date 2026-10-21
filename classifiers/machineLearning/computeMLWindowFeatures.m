function [values, names] = computeMLWindowFeatures( ...
        accel, gyro, emg, time_s, handIndex, cfg)
% COMPUTEMLWINDOWFEATURES Calculate interpretable IMU and EMG descriptors.

    arguments
        accel (:,3) double {mustBeFinite}
        gyro (:,3) double {mustBeFinite}
        emg (:,:) double {mustBeFinite, mustBeNonnegative}
        time_s (:,1) double {mustBeFinite}
        handIndex (1,1) double {mustBeInteger, mustBePositive}
        cfg (1,1) struct
    end
    if size(accel,1) ~= size(gyro,1) || size(accel,1) ~= size(emg,1) || ...
            size(accel,1) ~= numel(time_s) || size(accel,1) < 3
        error('MyoDrum:InvalidMLFeatureWindow', ...
            'IMU, EMG and timestamp window lengths must match.');
    end
    if handIndex > numel(cfg.IMU.HandNames)
        error('MyoDrum:InvalidMLFeatureHand', ...
            'Hand index %d is not configured.', handIndex);
    end

    [accelValues, accelNames] = axisStatistics(accel, "Accel");
    [gyroValues, gyroNames] = axisStatistics(gyro, "Gyro");
    accelMagnitude = vecnorm(accel,2,2);
    gyroMagnitude = vecnorm(gyro,2,2);
    dynamicAccel = abs(accelMagnitude-1);
    dt = diff(time_s);
    jerkMagnitude = vecnorm(diff(accel,1,1)./dt,2,2);
    angularAccelMagnitude = vecnorm(diff(gyro,1,1)./dt,2,2);
    magnitudeValues = [mean(accelMagnitude), std(accelMagnitude), ...
        max(accelMagnitude), range(accelMagnitude), ...
        rootMeanSquare(dynamicAccel), max(dynamicAccel), ...
        mean(gyroMagnitude), std(gyroMagnitude), max(gyroMagnitude), ...
        rootMeanSquare(gyroMagnitude), rootMeanSquare(jerkMagnitude), ...
        max(jerkMagnitude), rootMeanSquare(angularAccelMagnitude), ...
        max(angularAccelMagnitude)];
    magnitudeNames = ["AccelMagMean","AccelMagStd","AccelMagMax", ...
        "AccelMagRange","DynamicAccelRms","DynamicAccelMax", ...
        "GyroMagMean","GyroMagStd","GyroMagMax","GyroMagRms", ...
        "JerkMagRms","JerkMagMax","AngularAccelMagRms", ...
        "AngularAccelMagMax"];

    handName = cfg.IMU.HandNames(handIndex);
    activeChannels = cfg.EMG.ActiveChannelMask(:);
    if size(emg,2) ~= numel(activeChannels)
        error('MyoDrum:InvalidMLFeatureChannelMap', ...
            'EMG data width does not match the configured stream channels.');
    end
    sameChannels = find(activeChannels & cfg.EMG.Hands == handName);
    otherChannels = find(activeChannels & cfg.EMG.Hands ~= handName);
    if numel(sameChannels) < cfg.Fusion.RequiredActiveChannels
        error('MyoDrum:InvalidMLFeatureChannelMap', ...
            'The active hand has too few configured EMG channels.');
    end
    emgValues = zeros(1,6*numel(sameChannels));
    emgNames = strings(1,6*numel(sameChannels));
    cursor = 0;
    for channel = 1:numel(sameChannels)
        signal = emg(:,sameChannels(channel));
        current = [mean(signal), std(signal), max(signal), ...
            rootMeanSquare(signal), trapz(time_s,signal), ...
            mean(signal >= cfg.Fusion.EMGThresholdNormalized)];
        currentNames = "SameEMG" + channel + ...
            ["Mean","Std","Max","Rms","Area","ActiveFraction"];
        emgValues(cursor+(1:6)) = current;
        emgNames(cursor+(1:6)) = currentNames;
        cursor = cursor + 6;
    end
    otherValues = zeros(1,2*numel(otherChannels));
    otherNames = strings(1,2*numel(otherChannels));
    for channel = 1:numel(otherChannels)
        signal = emg(:,otherChannels(channel));
        at = 2*(channel-1) + (1:2);
        otherValues(at) = [max(signal), rootMeanSquare(signal)];
        otherNames(at) = "OtherEMG" + channel + ["Max", "Rms"];
    end

    gravityRows = 1:max(1,round(0.20*size(accel,1)));
    gravity = mean(accel(gravityRows,:),1);
    gravity = gravity / max(norm(gravity), eps);
    [~, peakGyroRow] = max(gyroMagnitude);
    postureValues = [gravity, gyro(peakGyroRow,:)];
    postureNames = ["StartGravityX","StartGravityY","StartGravityZ", ...
        "PeakGyroX","PeakGyroY","PeakGyroZ"];

    values = [accelValues, gyroValues, magnitudeValues, emgValues, ...
        otherValues, postureValues];
    names = [accelNames, gyroNames, magnitudeNames, emgNames, ...
        otherNames, postureNames];
    if any(~isfinite(values))
        error('MyoDrum:NonfiniteMLFeature', ...
            'A feature window produced a nonfinite value.');
    end
end

function [values, names] = axisStatistics(signal, prefix)
    axes = ["X","Y","Z"];
    statistics = {mean(signal,1), std(signal,0,1), ...
        min(signal,[],1), max(signal,[],1)};
    suffixes = ["Mean","Std","Min","Max"];
    values = zeros(1,12);
    names = strings(1,12);
    cursor = 0;
    for statistic = 1:numel(statistics)
        values(cursor+(1:3)) = statistics{statistic};
        names(cursor+(1:3)) = prefix + suffixes(statistic) + axes;
        cursor = cursor + 3;
    end
end

function value = rootMeanSquare(signal)
    value = sqrt(mean(signal.^2));
end
