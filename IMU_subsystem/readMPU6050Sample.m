function [sample, state] = readMPU6050Sample(sensor, options)
% READMPU6050SAMPLE Read one MPU6050 frame in robust or live mode.

    arguments
        sensor (1,1) struct
        options.Mode (1,1) string {mustBeMember(options.Mode, ...
            ["robust", "live"])} = "robust"
        options.State (1,1) struct = struct
        options.MaxConsecutiveFailures (1,1) double ...
            {mustBeInteger, mustBePositive} = 10
    end

    state = options.State;
    if ~isfield(state, 'lastValidSample')
        state.lastValidSample = [];
    end
    if ~isfield(state, 'consecutiveFailures')
        state.consecutiveFailures = 0;
    end

    if options.Mode == "live"
        maxAttempts = 3;
        retryPauseSeconds = 0.002;
    else
        maxAttempts = 20;
        retryPauseSeconds = 0.02;
    end
    raw = uint8([]);
    for attempt = 1:maxAttempts
        try
            raw = readRegister(sensor.dev, hex2dec('3B'), 14, 'uint8');
        catch
            raw = uint8([]);
        end
        raw = raw(:).';
        if numel(raw) == 14 && any(raw ~= 0)
            break;
        end

        if options.Mode == "robust" && (attempt == 5 || attempt == 12)
            try
                reapplyConfiguration(sensor);
            catch
                % Continue retrying; the final error contains the address.
            end
        end
        if attempt < maxAttempts
            pause(retryPauseSeconds);
        end
    end

    if numel(raw) ~= 14 || all(raw == 0)
        if options.Mode == "live" && ~isempty(state.lastValidSample)
            state.consecutiveFailures = state.consecutiveFailures + 1;
            if state.consecutiveFailures >= options.MaxConsecutiveFailures
                error('MyoDrum:InvalidIMUFrame', ...
                    ['MPU6050 at %s failed %d consecutive live reads. ' ...
                     'Check power, ground, SDA, SCL, and AD0.'], ...
                    sensor.address, state.consecutiveFailures);
            end
            sample = state.lastValidSample;
            sample.valid = false;
            sample.retries = maxAttempts;
            return;
        end
        error('MyoDrum:InvalidIMUFrame', ...
            ['MPU6050 at %s returned no valid frame after %d attempts. ' ...
             'Check power, ground, SDA, SCL, and AD0.'], ...
            sensor.address, maxAttempts);
    end

    gyroLsbPerDps = 131;
    if isfield(sensor, 'gyroLsbPerDps') && ~isempty(sensor.gyroLsbPerDps)
        gyroLsbPerDps = sensor.gyroLsbPerDps;
    end
    sample = decodeMPU6050Frame(raw, gyroLsbPerDps);
    sample.valid = true;
    sample.retries = attempt - 1;
    state.lastValidSample = sample;
    state.consecutiveFailures = 0;
end

function reapplyConfiguration(sensor)
    for registerIndex = 1:size(sensor.initRegs, 1)
        writeRegister(sensor.dev, sensor.initRegs(registerIndex, 1), ...
            uint8(sensor.initRegs(registerIndex, 2)), 'uint8');
        if registerIndex == 1
            pause(0.10);
        end
    end
end
