function [biases, noises, source] = resolveGyroBias( ...
        measureFunction, cacheFile, policy, maxAgeHours, forceMeasurement)
% RESOLVEGYROBIAS Select a fresh measurement or a validated cached bias.

    arguments
        measureFunction (1,1) function_handle
        cacheFile (1,1) string
        policy (1,1) string {mustBeMember(policy, ["fresh", "cached"])}
        maxAgeHours (1,1) double {mustBeFinite, mustBeNonnegative}
        forceMeasurement (1,1) logical = false
    end

    if policy == "cached" && ~forceMeasurement && isfile(cacheFile)
        saved = load(cacheFile, 'gyroCalibration');
        if isfield(saved, 'gyroCalibration') && ...
                cacheIsValid(saved.gyroCalibration, maxAgeHours)
            biases = saved.gyroCalibration.biasDps;
            noises = saved.gyroCalibration.noiseDps;
            source = "cached";
            return;
        end
        warning('MyoDrum:InvalidStaticCalibration', ...
            'Saved gyro calibration is invalid or expired and will be replaced.');
    end

    [biases, noises] = measureFunction();
    validateArrays(biases, noises);
    gyroCalibration.biasDps = biases;
    gyroCalibration.noiseDps = noises;
    gyroCalibration.createdAt = string(datetime('now', 'TimeZone', 'local'));
    gyroCalibration.schemaVersion = "1.1";
    outputFolder = fileparts(cacheFile);
    if ~isempty(outputFolder) && ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    save(cacheFile, 'gyroCalibration');
    source = "measured";
end

function valid = cacheIsValid(calibration, maxAgeHours)
    valid = isfield(calibration, 'biasDps') && ...
        isfield(calibration, 'noiseDps') && ...
        isequal(size(calibration.biasDps), [2, 3]) && ...
        isequal(size(calibration.noiseDps), [2, 3]) && ...
        all(isfinite(calibration.biasDps), 'all') && ...
        all(isfinite(calibration.noiseDps), 'all') && ...
        isfield(calibration, 'createdAt');
    if ~valid
        return;
    end
    try
        createdAt = datetime(calibration.createdAt);
        valid = isscalar(createdAt) && ~isnat(createdAt) && ...
            hours(datetime('now', 'TimeZone', createdAt.TimeZone) - createdAt) ...
            <= maxAgeHours;
    catch
        valid = false;
    end
end

function validateArrays(biases, noises)
    if ~isequal(size(biases), [2, 3]) || ...
            ~isequal(size(noises), [2, 3]) || ...
            any(~isfinite(biases), 'all') || any(~isfinite(noises), 'all')
        error('MyoDrum:InvalidGyroCalibration', ...
            'Measured gyro bias and noise must be finite 2-by-3 arrays.');
    end
end
