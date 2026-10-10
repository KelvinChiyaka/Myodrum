function report = importLegacyIMUCalibration(projectRoot)
% IMPORTLEGACYIMUCALIBRATION Import mainIMU calibration into the new cache.
%
% This migration is for calibration MAT-files produced by the former
% IMU_subsystem/mainIMU.m workflow. It does not remeasure either sensor.

    arguments
        projectRoot (1,1) string = string(fileparts( ...
            fileparts(mfilename('fullpath'))))
    end

    sourceFolder = fullfile(projectRoot, 'IMU_subsystem');
    destinationFolder = fullfile(projectRoot, 'data', 'metadata', 'calibration');
    if ~isfolder(destinationFolder)
        mkdir(destinationFolder);
    end

    report.sixPoseImported = false;
    report.staticImported = false;
    report.destinationFolder = string(destinationFolder);

    sixPoseFile = fullfile(sourceFolder, 'sixPoseCalibration.mat');
    if isfile(sixPoseFile)
        loaded = load(sixPoseFile, 'static');
        if ~isfield(loaded, 'static') || ...
                ~isfield(loaded.static, 'accelOffset') || ...
                ~isfield(loaded.static, 'accelScale') || ...
                ~isequal(size(loaded.static.accelOffset), [2, 3]) || ...
                ~isequal(size(loaded.static.accelScale), [2, 3])
            error('MyoDrum:InvalidLegacySixPose', ...
                'Legacy six-pose file has an unexpected schema.');
        end
        addresses = ["68", "69"];
        for sensorIndex = 1:2
            calibration.accelOffsetG = ...
                loaded.static.accelOffset(sensorIndex, :);
            calibration.accelScale = ...
                loaded.static.accelScale(sensorIndex, :);
            calibration.schemaVersion = "1.0-migrated";
            calibration.createdAt = string(datetime('now', 'TimeZone', 'local'));
            calibration.sourceFile = string(sixPoseFile);
            if isfield(loaded.static, 'gyroBias')
                calibration.legacyGyroBiasDps = ...
                    loaded.static.gyroBias(sensorIndex, :);
            end
            destinationFile = fullfile(destinationFolder, ...
                "mpu6050_0x" + addresses(sensorIndex) + ...
                "_accelCalibration.mat");
            save(destinationFile, 'calibration');
            clear calibration;
        end
        report.sixPoseImported = true;
    end

    staticFile = fullfile(sourceFolder, 'staticCalibration.mat');
    if isfile(staticFile)
        loaded = load(staticFile, 'static');
        if ~isfield(loaded, 'static') || ...
                ~isfield(loaded.static, 'gyroBias') || ...
                ~isequal(size(loaded.static.gyroBias), [2, 3])
            error('MyoDrum:InvalidLegacyStatic', ...
                'Legacy static-calibration file has an unexpected schema.');
        end
        gyroCalibration.biasDps = loaded.static.gyroBias;
        gyroCalibration.noiseDps = nan(2, 3);
        gyroCalibration.schemaVersion = "1.0-migrated";
        gyroCalibration.createdAt = string(datetime('now', 'TimeZone', 'local'));
        gyroCalibration.sourceFile = string(staticFile);
        destinationFile = fullfile(destinationFolder, ...
            'twoIMUStaticGyroCalibration.mat');
        save(destinationFile, 'gyroCalibration');
        report.staticImported = true;
    end

    fprintf('Legacy six-pose imported: %s\n', ...
        string(report.sixPoseImported));
    fprintf('Legacy static calibration imported: %s\n', ...
        string(report.staticImported));
    fprintf('Calibration cache:\n  %s\n', destinationFolder);
end
