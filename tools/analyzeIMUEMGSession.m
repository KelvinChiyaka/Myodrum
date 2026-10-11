function report = analyzeIMUEMGSession(sessionId, projectRoot)
% ANALYZEIMUEMGSESSION Summarize synchronized fusion quality and timing.
    arguments
        sessionId (1,1) string
        projectRoot (1,1) string = string(fileparts(fileparts(mfilename('fullpath'))))
    end
    rawFolder = fullfile(projectRoot, 'data', 'raw', sessionId);
    data = load(fullfile(rawFolder, 'imuEmgStream.mat'));
    calibration = load(fullfile(rawFolder, 'emgConfirmationCalibration.mat'));
    profile = calibration.emgCalibration.profile;
    names = ["LEFT_FOREARM"; "LEFT_TRICEPS"; ...
        "RIGHT_FOREARM"; "RIGHT_TRICEPS"];
    intervals = diff(data.time_s);
    report.duration_s = range(data.time_s);
    report.rate_Hz = 1 / median(intervals);
    report.maxInterval_s = max(intervals);
    report.invalidFrames = sum(~data.frameValid, 1);
    report.nonzeroStatusFrames = sum(data.deviceStatus ~= 0);
    report.emgQuality = table(names, min(data.emgVoltage_V)', ...
        median(data.emgVoltage_V)', localPercentile(data.emgVoltage_V, 95)', ...
        max(data.emgVoltage_V)', max(data.emgNormalized)', ...
        'VariableNames', {'Channel','MinV','MedianV','P95V','MaxV', ...
        'MaxNormalized'});
    report.emgCorrelation = corr(data.emgVoltage_V, 'Rows', 'complete');
    handReports = cell(1, 2);
    for hand = 1:2
        item = struct;
        indices = find(data.imuCandidateHit(:, hand));
        confirmed = data.emgConfirmedHit(indices, hand);
        scores = data.emgConfirmationScore(indices, hand);
        item.candidates = numel(indices);
        item.confirmed = sum(confirmed);
        item.confirmationRate = mean(confirmed);
        item.scoreP10 = localPercentile(scores, 10);
        item.scoreMedian = median(scores);
        item.scoreP90 = localPercentile(scores, 90);
        item.rejectedScoreMedian = median(scores(~confirmed));
        item.rejectedScoreMax = max(scores(~confirmed));
        item.targetCounts = [sum(data.targetIndex(:, hand)==1), ...
            sum(data.targetIndex(:, hand)==2), ...
            sum(data.targetIndex(:, hand)==3)];
        if numel(indices) > 1
            gaps = diff(data.time_s(indices));
            item.minimumCandidateGap_s = min(gaps);
            item.gapsUnder350ms = sum(gaps < 0.35);
        else
            item.minimumCandidateGap_s = NaN;
            item.gapsUnder350ms = 0;
        end
        channels = (1:2) + (hand-1)*2;
        futurePeak = zeros(numel(indices), 1);
        crossingDelay = nan(numel(indices), 1);
        for index = 1:numel(indices)
            last = min(size(data.emgNormalized,1), indices(index)+60);
            segment = max(data.emgNormalized(indices(index):last,channels),[],2);
            futurePeak(index) = max(segment);
            crossing = find(segment >= 0.20, 1);
            if ~isempty(crossing)
                crossingDelay(index) = data.time_s(indices(index)+crossing-1) ...
                    - data.time_s(indices(index));
            end
        end
        rejected = ~confirmed;
        item.rejectedRescuedWithin300ms = sum(futurePeak(rejected) >= 0.20);
        item.rejectedCount = sum(rejected);
        finiteDelay = crossingDelay(rejected & isfinite(crossingDelay));
        if isempty(finiteDelay)
            item.rescueDelayMedian_s = NaN;
            item.rescueDelayP90_s = NaN;
        else
            item.rescueDelayMedian_s = median(finiteDelay);
            item.rescueDelayP90_s = localPercentile(finiteDelay, 90);
        end
        handReports{hand} = item;
    end
    report.hands = [handReports{:}];
    report.sameFrameCandidates = sum(all(data.imuCandidateHit,2));
    report.sameFrameConfirmed = sum(all(data.emgConfirmedHit,2));
    report.profile = profile;
end

function result = localPercentile(values, percentage)
    values = sort(values, 1);
    if isempty(values), result = NaN; return; end
    position = 1 + (size(values,1)-1)*percentage/100;
    low=floor(position); high=ceil(position); fraction=position-low;
    result=values(low,:)*(1-fraction)+values(high,:)*fraction;
end
