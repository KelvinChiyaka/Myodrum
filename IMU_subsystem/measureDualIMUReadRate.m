function sampleRateHz = measureDualIMUReadRate(sensors, cycleCount)
% MEASUREDUALIMUREADRATE Measure the achieved complete-pair acquisition rate.

    arguments
        sensors (1,2) struct
        cycleCount (1,1) double {mustBeInteger, mustBePositive} = 25
    end

    for warmupIndex = 1:3
        readMPU6050Sample(sensors(1));
        readMPU6050Sample(sensors(2));
    end

    timer = tic;
    for cycleIndex = 1:cycleCount
        readMPU6050Sample(sensors(1));
        readMPU6050Sample(sensors(2));
    end
    sampleRateHz = cycleCount / toc(timer);
    if ~isfinite(sampleRateHz) || sampleRateHz < 5
        error('MyoDrum:ReadRateTooLow', ...
            'Combined per-IMU MATLAB read rate is only %.2f Hz.', sampleRateHz);
    end
    fprintf('Measured complete two-IMU cycle rate: %.2f Hz per IMU.\n', ...
        sampleRateHz);
end
