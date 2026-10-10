function mainEMGRecord(options)
% MAINEMGRECORD Record four standalone EMG envelope channels from the Mega.

    arguments
        options.Config (1,1) struct = struct
        options.Reader = []
        options.DurationSeconds (1,1) double {mustBeNonnegative} = inf
        options.SkipRest (1,1) logical = false
        options.ShowPlot (1,1) logical = true
        options.GainNotes (:,1) string = strings(0, 1)
        options.Notes (1,1) string = ""
        options.MarkerProvider = []
    end

    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(projectRoot, 'common'));
    addpath(fullfile(projectRoot, 'EMG_subsystem'));
    if isempty(fieldnames(options.Config))
        cfg = myoDrumConfig(projectRoot);
    else
        cfg = options.Config;
    end
    pins = string(cfg.EMG.AnalogPins(:));
    channelNames = string(cfg.EMG.ChannelNames(:));
    hands = string(cfg.EMG.Hands(:));
    channelCount = numel(pins);
    validateChannelConfiguration(pins, channelNames, hands);

    gainNotes = options.GainNotes;
    if isempty(gainNotes)
        gainNotes = repmat("unrecorded", channelCount, 1);
    elseif numel(gainNotes) ~= channelCount
        error('MyoDrum:InvalidEMGGainNotes', ...
            'GainNotes must contain one entry per configured EMG channel.');
    end

    sessionId = "session_" + string(datetime('now', ...
        'Format', 'yyyyMMdd_HHmmss'));
    rawFolder = fullfile(cfg.Paths.RawData, sessionId);
    metadataFolder = fullfile(cfg.Paths.Metadata, 'sessions', sessionId);
    processedFolder = fullfile(cfg.Paths.ProcessedData, sessionId);
    mkdir(rawFolder);
    mkdir(metadataFolder);
    rawFile = string(fullfile(rawFolder, 'emgAcquisition.mat'));
    metadataFile = string(fullfile(metadataFolder, 'sessionMetadata.mat'));

    metadata = initialMetadata(sessionId, cfg, rawFile, channelNames, ...
        pins, hands, gainNotes, options.Notes);
    logger = EMGSessionLogger(rawFile, metadataFile, metadata, channelCount, ...
        cfg.Logging.BufferSamples);
    connection = struct('board', [], 'readVoltage', []);
    fig = gobjects(0);
    plotState = struct;
    sessionTimer = tic;
    try
        if isempty(options.Reader)
            connection = connectEMGChannels(cfg);
            reader = connection.readVoltage;
        else
            if ~isa(options.Reader, 'function_handle')
                error('MyoDrum:InvalidEMGReader', ...
                    'Reader must be empty or a function handle accepting a pin.');
            end
            reader = options.Reader;
        end
    catch connectionError
        logger.close();
        rethrow(connectionError);
    end

    if options.ShowPlot
        [fig, plotState] = createLivePlot(channelNames, hands);
    end
    cleanup = onCleanup(@()shutdownEMG(logger, connection.board, fig, cfg, ...
        metadataFolder, processedFolder, channelNames, hands, ...
        options.ShowPlot));

    fprintf('\nMyoDrum: standalone EMG envelope recorder\n');
    fprintf('Session: %s\n', sessionId);
    preflight = acquireTimedBlock(reader, pins, sessionTimer, ...
        cfg.EMG.PreflightSeconds);
    printPreflight(preflight, channelNames, pins, cfg.EMG.Quality);

    restBaseline = [];
    totalSetCount = 0;
    if ~options.SkipRest
        input(sprintf(['\nRelax every monitored muscle. Press Enter to record ' ...
            'a %.1f-second rest baseline...'], cfg.EMG.RestSeconds), 's');
        restStart = toc(sessionTimer);
        while toc(sessionTimer) - restStart < cfg.EMG.RestSeconds
            [voltages, readTimes] = readEMGSet(reader, pins, sessionTimer);
            logger.append(mean(readTimes), voltages, diff(readTimes), ...
                uint8(1), uint8(0));
            totalSetCount = totalSetCount + 1;
            updateLivePlot(fig, plotState, mean(readTimes), voltages, ...
                cfg, totalSetCount);
        end
        logger.flush();
        recorded = load(rawFile, 'emgVoltage_V', 'phase');
        restBaseline = computeRestBaseline( ...
            recorded.emgVoltage_V(recorded.phase == 1, :));
        printRestBaseline(restBaseline, channelNames, cfg.EMG.Quality);
    end

    fprintf('\nFREE RECORDING\n');
    fprintf('H = intended hit, X = unintended motion, R = reference squeeze.\n');
    if options.ShowPlot
        fprintf('Press Q or close the window to stop and save.\n\n');
    else
        fprintf('Headless recording stops after DurationSeconds.\n\n');
    end
    freeStart = toc(sessionTimer);
    freeSetIndex = 0;
    while shouldContinue(fig, options.ShowPlot, sessionTimer, freeStart, ...
            options.DurationSeconds)
        freeSetIndex = freeSetIndex + 1;
        [voltages, readTimes] = readEMGSet(reader, pins, sessionTimer);
        markerCode = nextMarker(fig, options.MarkerProvider, ...
            freeSetIndex, mean(readTimes));
        logger.append(mean(readTimes), voltages, diff(readTimes), ...
            uint8(2), markerCode);
        totalSetCount = totalSetCount + 1;
        updateLivePlot(fig, plotState, mean(readTimes), voltages, ...
            cfg, totalSetCount);
    end

    logger.flush();
    recorded = load(rawFile, 'time_s', 'emgVoltage_V', ...
        'setDuration_s', 'phase', 'marker');
    finalFields = buildFinalMetadata(recorded, restBaseline, cfg);
    logger.updateMetadata(finalFields);
    logger.CompletedNormally = true;
    clear cleanup;
end

function shutdownEMG(logger, board, fig, cfg, metadataFolder, ...
        processedFolder, channelNames, hands, writeSummaryFigure)
    try
        logger.close();
    catch saveError
        warning('MyoDrum:FinalEMGSaveFailed', ...
            'Could not flush the final EMG buffer: %s', saveError.message);
    end
    if ~isempty(fig) && all(isgraphics(fig))
        delete(fig);
    end
    if ~isempty(board)
        board = []; %#ok<NASGU> Release the captured Arduino object.
    end
    printSavedSummary(logger, cfg, metadataFolder);
    if writeSummaryFigure
        try
            writeOverview(logger.RawFile, processedFolder, channelNames, hands);
        catch overviewError
            warning('MyoDrum:EMGOverviewFailed', ...
                'EMG data were saved, but the overview failed: %s', ...
                overviewError.message);
        end
    end
end

function metadata = initialMetadata(sessionId, cfg, rawFile, channelNames, ...
        pins, hands, gainNotes, notes)
    metadata.sessionId = sessionId;
    metadata.startedAt = string(datetime('now', 'TimeZone', 'local'));
    metadata.endedAt = "";
    metadata.mode = "emg-only";
    metadata.config = cfg;
    metadata.rawDataFile = rawFile;
    metadata.channelNames = channelNames;
    metadata.analogPins = pins;
    metadata.hands = hands;
    metadata.phaseNames = ["REST"; "FREE"];
    metadata.markerCodes = struct('none', uint8(0), ...
        'intendedHit', uint8(1), 'unintendedMotion', uint8(2), ...
        'referenceSqueeze', uint8(3));
    metadata.measuredSetRateHz = NaN;
    metadata.setIntervalStats = struct('median', NaN, 'p95', NaN, ...
        'p99', NaN, 'max', NaN);
    metadata.restBaseline = [];
    metadata.quality = struct;
    metadata.channelGainNotes = gainNotes;
    metadata.operatorNotes = notes;
    metadata.rateWarning = "";
    metadata.loggedSamples = 0;
    metadata.completedNormally = false;
    metadata.matlabRelease = string(version('-release'));
    metadata.acquisitionMethod = "MATLAB arduino readVoltage";
    metadata.timestampSource = "host tic/toc; not the IMU device clock";
end

function validateChannelConfiguration(pins, names, hands)
    if isempty(pins) || numel(names) ~= numel(pins) || ...
            numel(hands) ~= numel(pins) || numel(unique(pins)) ~= numel(pins)
        error('MyoDrum:InvalidEMGConfiguration', ...
            'EMG pins must be unique and match the channel-name and hand counts.');
    end
end

function block = acquireTimedBlock(reader, pins, sessionTimer, durationSeconds)
    capacity = 32;
    block = nan(capacity, numel(pins));
    count = 0;
    startTime = toc(sessionTimer);
    while toc(sessionTimer) - startTime < durationSeconds || count == 0
        count = count + 1;
        if count > size(block, 1)
            block = [block; nan(capacity, numel(pins))]; %#ok<AGROW>
        end
        block(count, :) = readEMGSet(reader, pins, sessionTimer);
    end
    block = block(1:count, :);
end

function printPreflight(values, names, pins, qualityCfg)
    medians = median(values, 1, 'omitnan')';
    ranges = (max(values, [], 1, 'omitnan') - ...
        min(values, [], 1, 'omitnan'))';
    fprintf('\nEMG PREFLIGHT\n');
    disp(table(names, pins, medians, ranges, 'VariableNames', ...
        {'Channel', 'Pin', 'Median_V', 'Range_V'}));
    for channelIndex = 1:numel(names)
        if any(~isfinite(values(:, channelIndex)))
            warning('MyoDrum:NonfiniteEMG', ...
                '%s produced a non-finite preflight reading.', names(channelIndex));
        elseif max(values(:, channelIndex)) >= qualityCfg.RailVolts
            warning('MyoDrum:RailedEMG', ...
                '%s is at the voltage rail; reduce its sensor gain.', names(channelIndex));
        elseif all(abs(values(:, channelIndex)) < eps)
            fprintf(['%s is 0 V at rest: normal for an envelope; confirm it ' ...
                'rises on a squeeze.\n'], names(channelIndex));
        end
    end
end

function printRestBaseline(baseline, names, qualityCfg)
    fprintf('\nREST BASELINE\n');
    disp(table(names, baseline.median_V', baseline.robustSigma_V', ...
        'VariableNames', {'Channel', 'Median_V', 'RobustSigma_V'}));
    noisy = baseline.robustSigma_V > qualityCfg.RestSigmaWarnVolts;
    if any(noisy)
        warning('MyoDrum:NoisyEMGRest', ...
            'High resting variability on: %s.', strjoin(names(noisy), ', '));
    end
end

function [fig, state] = createLivePlot(names, hands)
    fig = figure('Name', 'MyoDrum EMG recorder', 'NumberTitle', 'off', ...
        'Color', 'w', 'KeyPressFcn', @keyPressed, ...
        'CloseRequestFcn', @closeRequested);
    layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact');
    state.lines = gobjects(numel(names), 1);
    handOrder = ["LEFT", "RIGHT"];
    for handIndex = 1:numel(handOrder)
        ax = nexttile(layout);
        hold(ax, 'on');
        channels = find(hands == handOrder(handIndex));
        for channelIndex = channels'
            state.lines(channelIndex) = animatedline(ax, ...
                'DisplayName', names(channelIndex), 'LineWidth', 1.1);
        end
        grid(ax, 'on');
        ylim(ax, [0, 5]);
        ylabel(ax, 'Envelope (V)');
        title(ax, handOrder(handIndex) + " ARM");
        legend(ax, 'Location', 'northwest');
    end
    xlabel(ax, 'Session time (s)');
    setappdata(fig, 'MyoDrumRunning', true);
    setappdata(fig, 'PendingMarker', uint8(0));
    drawnow;
end

function keyPressed(source, event)
    switch lower(event.Key)
        case 'q'
            setappdata(source, 'MyoDrumRunning', false);
        case 'h'
            setappdata(source, 'PendingMarker', uint8(1));
        case 'x'
            setappdata(source, 'PendingMarker', uint8(2));
        case 'r'
            setappdata(source, 'PendingMarker', uint8(3));
    end
end

function closeRequested(source, ~)
    setappdata(source, 'MyoDrumRunning', false);
end

function result = shouldContinue(fig, showPlot, timer, startTime, duration)
    withinDuration = toc(timer) - startTime < duration;
    if showPlot
        result = isgraphics(fig) && getappdata(fig, 'MyoDrumRunning') && ...
            withinDuration;
    else
        result = withinDuration;
    end
end

function marker = nextMarker(fig, provider, setIndex, timeSeconds)
    marker = uint8(0);
    if ~isempty(provider)
        marker = uint8(provider(setIndex, timeSeconds));
    elseif isgraphics(fig)
        marker = uint8(getappdata(fig, 'PendingMarker'));
        setappdata(fig, 'PendingMarker', uint8(0));
    end
    if ~isscalar(marker) || marker > 3
        error('MyoDrum:InvalidEMGMarker', ...
            'MarkerProvider must return a scalar marker from 0 through 3.');
    end
end

function updateLivePlot(fig, state, timeSeconds, voltages, cfg, sampleNumber)
    if isempty(fig) || ~all(isgraphics(fig)) || ...
            mod(sampleNumber, cfg.EMG.PlotEverySets) ~= 0
        return;
    end
    for channelIndex = 1:numel(state.lines)
        addpoints(state.lines(channelIndex), timeSeconds, voltages(channelIndex));
        ax = ancestor(state.lines(channelIndex), 'axes');
        xlim(ax, [max(0, timeSeconds - cfg.EMG.LiveWindowSeconds), ...
            max(cfg.EMG.LiveWindowSeconds, timeSeconds)]);
    end
    drawnow limitrate;
end

function fields = buildFinalMetadata(recorded, restBaseline, cfg)
    intervals = diff(recorded.time_s);
    intervals = intervals(isfinite(intervals) & intervals > 0);
    if isempty(intervals)
        rateHz = NaN;
        stats = struct('median', NaN, 'p95', NaN, 'p99', NaN, 'max', NaN);
    else
        stats = struct('median', median(intervals), ...
            'p95', percentileValue(intervals, 95), ...
            'p99', percentileValue(intervals, 99), 'max', max(intervals));
        rateHz = 1 / stats.median;
    end
    quality = summarizeEMGQuality(recorded.emgVoltage_V, ...
        restBaseline, cfg.EMG.Quality);
    rateWarning = "";
    if ~isfinite(rateHz) || rateHz < cfg.EMG.MinRecommendedSetRateHz
        rateWarning = sprintf(['Recorded at %.2f Hz. Burst peaks are ' ...
            'undersampled lower bounds and timing is coarse; this recording ' ...
            'is for amplitude and placement checks, not hit-timing analysis.'], ...
            rateHz);
    end
    fields.measuredSetRateHz = rateHz;
    fields.setIntervalStats = stats;
    fields.restBaseline = restBaseline;
    fields.quality = quality;
    fields.rateWarning = string(rateWarning);
end

function printSavedSummary(logger, cfg, metadataFolder)
    fprintf('\nEMG recording stopped. Logged %d channel sets.\n', ...
        logger.LoggedSamples);
    if ~isfile(logger.RawFile)
        fprintf('No EMG samples were saved.\n');
        return;
    end
    raw = load(logger.RawFile, 'time_s', 'emgVoltage_V');
    saved = load(logger.MetadataFile, 'metadata');
    metadata = saved.metadata;
    fprintf('Measured set rate: %.2f Hz.\n', metadata.measuredSetRateHz);
    stats = metadata.setIntervalStats;
    fprintf('Set interval median/p95/p99/max: %.3f / %.3f / %.3f / %.3f s.\n', ...
        stats.median, stats.p95, stats.p99, stats.max);
    quality = summarizeEMGQuality(raw.emgVoltage_V, ...
        metadata.restBaseline, cfg.EMG.Quality);
    disp(table(metadata.channelNames, quality.minimum_V', ...
        quality.median_V', quality.percentile95_V', quality.maximum_V', ...
        quality.neverResponded', quality.railed', quality.noisyRest', ...
        'VariableNames', {'Channel', 'Min_V', 'Median_V', 'P95_V', 'Max_V', ...
        'NeverResponded', 'Railed', 'NoisyRest'}));
    if strlength(metadata.rateWarning) > 0
        warning('MyoDrum:LowEMGSetRate', '%s', metadata.rateWarning);
    end
    fprintf('Raw data:\n  %s\n', logger.RawFile);
    fprintf('Metadata:\n  %s\n', metadataFolder);
end

function writeOverview(rawFile, processedFolder, names, hands)
    if ~isfile(rawFile)
        return;
    end
    data = load(rawFile, 'time_s', 'emgVoltage_V', 'marker');
    mkdir(processedFolder);
    fig = figure('Visible', 'off', 'Color', 'w');
    cleanup = onCleanup(@()delete(fig));
    layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact');
    for handIndex = 1:2
        handName = ["LEFT", "RIGHT"];
        ax = nexttile(layout);
        channels = find(hands == handName(handIndex));
        plot(ax, data.time_s, data.emgVoltage_V(:, channels), ...
            'LineWidth', 0.8);
        hold(ax, 'on');
        markerRows = find(data.marker > 0);
        for row = markerRows'
            xline(ax, data.time_s(row), ':', string(data.marker(row)), ...
                'HandleVisibility', 'off');
        end
        ylim(ax, [0, 5]);
        grid(ax, 'on');
        ylabel(ax, 'Envelope (V)');
        title(ax, handName(handIndex) + " ARM");
        legend(ax, names(channels), 'Location', 'best');
    end
    xlabel(ax, 'Session time (s)');
    exportgraphics(fig, fullfile(processedFolder, 'emgOverview.png'));
    clear cleanup;
end

function value = percentileValue(values, percentage)
    values = sort(values(:));
    position = 1 + (numel(values) - 1) * percentage / 100;
    lowerIndex = floor(position);
    upperIndex = ceil(position);
    fraction = position - lowerIndex;
    value = values(lowerIndex) * (1 - fraction) + ...
        values(upperIndex) * fraction;
end
