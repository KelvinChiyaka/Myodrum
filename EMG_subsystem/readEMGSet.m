function [voltages, readTimes] = readEMGSet(reader, analogPins, sessionTimer)
% READEMGSET Read one fixed-order set of analog EMG envelope channels.

% readTimes contains the host times immediately before the first and after
% the final serial read. It is not a device-clock timestamp.

    arguments
        reader (1,1) function_handle
        analogPins (:,1) string
        sessionTimer
    end

    channelCount = numel(analogPins);
    voltages = zeros(1, channelCount);
    readTimes = zeros(1, 2);
    readTimes(1) = toc(sessionTimer);
    for channelIndex = 1:channelCount
        voltages(channelIndex) = reader(analogPins(channelIndex));
    end
    readTimes(2) = toc(sessionTimer);
end
