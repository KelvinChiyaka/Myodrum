function mainEMGStreamRecord(options)
% MAINEMGSTREAMRECORD Record firmware-streamed EMG envelope samples.

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
    names = string(cfg.EMG.ChannelNames(:));
    pins = string(cfg.EMG.AnalogPins(:));
    hands = string(cfg.EMG.Hands(:));
    channelCount = numel(names);
    gainNotes = options.GainNotes;
    if isempty(gainNotes)
        gainNotes = repmat("unrecorded", channelCount, 1);
    elseif numel(gainNotes) ~= channelCount
        error('MyoDrum:InvalidEMGGainNotes', ...
            'GainNotes must contain one entry per configured channel.');
    end

    sessionId = "session_" + string(datetime('now', ...
        'Format', 'yyyyMMdd_HHmmss'));
    rawFolder = fullfile(cfg.Paths.RawData, sessionId);
    metadataFolder = fullfile(cfg.Paths.Metadata, 'sessions', sessionId);
    mkdir(rawFolder);
    mkdir(metadataFolder);
    rawFile = string(fullfile(rawFolder, 'emgAcquisition.mat'));
    metadataFile = string(fullfile(metadataFolder, 'sessionMetadata.mat'));
    metadata = createMetadata(sessionId, cfg, rawFile, names, pins, hands, ...
        gainNotes, options.Notes);
    logger = EMGSessionLogger(rawFile, metadataFile, metadata, channelCount, ...
        cfg.Logging.BufferSamples);

    if isempty(options.Reader)
        reader = EMGStreamReader(cfg.Hardware.COMPort, cfg.EMG.StreamBaud);
    else
        reader = options.Reader;
    end
    fig = gobjects(0);
    lines = gobjects(0);
    if options.ShowPlot
        [fig, lines] = createStreamPlot(names, hands);
    end
    cleanup = onCleanup(@()shutdownStream(reader, logger, fig, cfg, ...
        metadataFolder));
    reader.start();

    fprintf('\nMyoDrum: high-rate EMG stream recorder\n');
    fprintf('Session: %s\n', sessionId);
    fprintf('Firmware target: %.0f complete sets/s at %.0f baud.\n', ...
        cfg.EMG.StreamRateHz, cfg.EMG.StreamBaud);
    preflight = collectPreview(reader, cfg.EMG.PreflightSeconds, cfg);
    printPreview(preflight, names, pins, cfg.EMG.Quality);

    restBaseline = [];
    lastPlotTime = -inf;
    if ~options.SkipRest
        input(sprintf(['\nRelax every monitored muscle. Press Enter to record ' ...
            'a %.1f-second rest baseline...'], cfg.EMG.RestSeconds), 's');
        phaseTimer = tic;
        while toc(phaseTimer) < cfg.EMG.RestSeconds
            frames = reader.readAvailableFrames();
            appendFrames(logger, frames, uint8(1), uint8(0), cfg);
            lastPlotTime = plotFrames(fig, lines, frames, cfg, lastPlotTime);
            pause(0.001);
        end
        logger.flush();
        saved = load(rawFile, 'emgVoltage_V', 'phase');
        restBaseline = computeRestBaseline( ...
            saved.emgVoltage_V(saved.phase == 1, :));
        disp(table(names, restBaseline.median_V', ...
            restBaseline.robustSigma_V', 'VariableNames', ...
            {'Channel', 'Median_V', 'RobustSigma_V'}));
    end

    fprintf('\nFREE RECORDING\n');
    fprintf('H = intended hit, X = unintended motion, R = reference squeeze.\n');
    fprintf('Press Q or close the window to stop and save.\n\n');
    phaseTimer = tic;
    freeFrameIndex = 0;
    while continueRecording(fig, options.ShowPlot, phaseTimer, ...
            options.DurationSeconds)
        frames = reader.readAvailableFrames();
        marker = uint8(0);
        if ~isempty(frames.sequence)
            freeFrameIndex = freeFrameIndex + 1;
            marker = obtainMarker(fig, options.MarkerProvider, ...
                freeFrameIndex, frames.time_s(1));
        end
        appendFrames(logger, frames, uint8(2), marker, cfg);
        lastPlotTime = plotFrames(fig, lines, frames, cfg, lastPlotTime);
        if isempty(frames.sequence)
            pause(0.001);
        end
    end
    reader.stop();
    logger.flush();
    saved = load(rawFile, 'time_s', 'emgVoltage_V');
    statistics = reader.getStatistics();
    final = summarizeSession(saved, restBaseline, statistics, cfg);
    logger.updateMetadata(final);
    logger.CompletedNormally = true;
    clear cleanup;
end

function metadata = createMetadata(sessionId, cfg, rawFile, names, pins, ...
        hands, gainNotes, notes)
    metadata.sessionId = sessionId;
    metadata.startedAt = string(datetime('now', 'TimeZone', 'local'));
    metadata.mode = "emg-stream";
    metadata.config = cfg;
    metadata.rawDataFile = rawFile;
    metadata.channelNames = names;
    metadata.analogPins = pins;
    metadata.hands = hands;
    metadata.phaseNames = ["REST"; "FREE"];
    metadata.markerCodes = struct('none', uint8(0), ...
        'intendedHit', uint8(1), 'unintendedMotion', uint8(2), ...
        'referenceSqueeze', uint8(3));
    metadata.channelGainNotes = gainNotes;
    metadata.operatorNotes = notes;
    metadata.restBaseline = [];
    metadata.quality = struct;
    metadata.measuredSetRateHz = NaN;
    metadata.setIntervalStats = struct;
    metadata.streamStatistics = struct;
    metadata.rateWarning = "";
    metadata.completedNormally = false;
    metadata.matlabRelease = string(version('-release'));
    metadata.acquisitionMethod = "Mega binary serial stream";
    metadata.timestampSource = "Arduino micros() device clock";
end

function values = collectPreview(reader, durationSeconds, cfg)
    capacity = max(1, ceil(durationSeconds * cfg.EMG.StreamRateHz * 1.2));
    values = zeros(capacity, numel(cfg.EMG.AnalogPins));
    count = 0;
    timer = tic;
    while toc(timer) < durationSeconds
        frames = reader.readAvailableFrames();
        if ~isempty(frames.adc)
            volts = adcToVolts(frames.adc, cfg);
            required = count + size(volts, 1);
            if required > size(values, 1)
                values = [values; zeros(capacity, size(values, 2))]; %#ok<AGROW>
            end
            values(count + (1:size(volts, 1)), :) = volts;
            count = required;
        else
            pause(0.001);
        end
    end
    values = values(1:count, :);
    if isempty(values)
        error('MyoDrum:NoEMGStreamFrames', ...
            ['No valid firmware packets arrived. Confirm that the streaming ' ...
             'sketch is uploaded and the configured baud rate matches.']);
    end
end

function appendFrames(logger, frames, phase, firstMarker, cfg)
    volts = adcToVolts(frames.adc, cfg);
    for row = 1:numel(frames.sequence)
        marker = uint8(0);
        if row == 1
            marker = firstMarker;
        end
        logger.append(frames.time_s(row), volts(row, :), 0, phase, marker, ...
            SequenceNumber=frames.sequence(row), ...
            DeviceStatus=frames.status(row));
    end
end

function volts = adcToVolts(adc, cfg)
    volts = double(adc) * cfg.EMG.ADCReferenceVolts / cfg.EMG.ADCMaxCount;
end

function printPreview(values, names, pins, qualityCfg)
    medians = median(values, 1)';
    ranges = (max(values, [], 1) - min(values, [], 1))';
    fprintf('\nEMG STREAM PREFLIGHT\n');
    disp(table(names, pins, medians, ranges, 'VariableNames', ...
        {'Channel', 'Pin', 'Median_V', 'Range_V'}));
    if any(max(values, [], 1) >= qualityCfg.RailVolts)
        warning('MyoDrum:RailedEMG', ...
            'At least one EMG channel reached the configured voltage rail.');
    end
end

function final = summarizeSession(saved, baseline, statistics, cfg)
    intervals = diff(saved.time_s);
    intervals = intervals(intervals > 0 & isfinite(intervals));
    if isempty(intervals)
        intervalStats = struct('median', NaN, 'p95', NaN, ...
            'p99', NaN, 'max', NaN);
    else
        intervalStats = struct('median', median(intervals), ...
            'p95', localPercentile(intervals, 95), ...
            'p99', localPercentile(intervals, 99), 'max', max(intervals));
    end
    quality = summarizeEMGQuality(saved.emgVoltage_V, baseline, ...
        cfg.EMG.Quality);
    warningText = "";
    if ~isfinite(statistics.measuredRateHz) || ...
            statistics.measuredRateHz < cfg.EMG.MinRecommendedSetRateHz
        warningText = sprintf('Recorded at %.2f Hz, below the recommended %.0f Hz.', ...
            statistics.measuredRateHz, cfg.EMG.MinRecommendedSetRateHz);
    end
    final.measuredSetRateHz = statistics.measuredRateHz;
    final.setIntervalStats = intervalStats;
    final.restBaseline = baseline;
    final.quality = quality;
    final.streamStatistics = statistics;
    final.rateWarning = string(warningText);
end

function shutdownStream(reader, logger, fig, cfg, metadataFolder)
    try
        reader.stop();
    catch
    end
    try
        logger.close();
    catch saveError
        warning('MyoDrum:FinalEMGSaveFailed', '%s', saveError.message);
    end
    if ~isempty(fig) && all(isgraphics(fig))
        delete(fig);
    end
    stats = reader.getStatistics();
    fprintf('\nHigh-rate EMG recording stopped. Logged %d sets.\n', ...
        logger.LoggedSamples);
    fprintf(['Stream rate %.2f Hz; dropped %d; CRC failures %d; ' ...
        'parse resets %d.\n'], stats.measuredRateHz, stats.droppedFrames, ...
        stats.checksumFailures, stats.parseResets);
    fprintf('Raw data:\n  %s\nMetadata:\n  %s\n', ...
        logger.RawFile, metadataFolder);
    if isfinite(stats.measuredRateHz) && ...
            stats.measuredRateHz < cfg.EMG.MinRecommendedSetRateHz
        warning('MyoDrum:LowEMGStreamRate', ...
            'The measured firmware stream rate is below %.0f Hz.', ...
            cfg.EMG.MinRecommendedSetRateHz);
    end
end

function [fig, lines] = createStreamPlot(names, hands)
    fig = figure('Name', 'MyoDrum fast EMG stream', 'NumberTitle', 'off', ...
        'Color', 'w', 'KeyPressFcn', @keyPressed, ...
        'CloseRequestFcn', @closeRequested);
    layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact');
    lines = gobjects(numel(names), 1);
    handNames = ["LEFT", "RIGHT"];
    for handIndex = 1:2
        ax = nexttile(layout);
        hold(ax, 'on');
        channels = find(hands == handNames(handIndex));
        for channel = channels'
            lines(channel) = animatedline(ax, 'DisplayName', names(channel));
        end
        ylim(ax, [0, 5]);
        grid(ax, 'on');
        legend(ax, 'Location', 'northwest');
        ylabel(ax, 'Envelope (V)');
        title(ax, handNames(handIndex) + " ARM");
    end
    setappdata(fig, 'MyoDrumRunning', true);
    setappdata(fig, 'PendingMarker', uint8(0));
end

function lastPlotTime = plotFrames(fig, lines, frames, cfg, lastPlotTime)
    if isempty(fig) || ~all(isgraphics(fig)) || isempty(frames.sequence) || ...
            frames.time_s(end) - lastPlotTime < cfg.EMG.StreamPlotEverySeconds
        return;
    end
    volts = adcToVolts(frames.adc, cfg);
    for channel = 1:numel(lines)
        addpoints(lines(channel), frames.time_s, volts(:, channel));
        ax = ancestor(lines(channel), 'axes');
        xlim(ax, [max(0, frames.time_s(end) - cfg.EMG.LiveWindowSeconds), ...
            max(cfg.EMG.LiveWindowSeconds, frames.time_s(end))]);
    end
    lastPlotTime = frames.time_s(end);
    drawnow limitrate;
end

function keyPressed(source, event)
    keys = struct('h', uint8(1), 'x', uint8(2), 'r', uint8(3));
    key = lower(event.Key);
    if strcmp(key, 'q')
        setappdata(source, 'MyoDrumRunning', false);
    elseif isfield(keys, key)
        setappdata(source, 'PendingMarker', keys.(key));
    end
end

function closeRequested(source, ~)
    setappdata(source, 'MyoDrumRunning', false);
end

function result = continueRecording(fig, showPlot, timer, duration)
    result = toc(timer) < duration;
    if showPlot
        result = result && isgraphics(fig) && ...
            getappdata(fig, 'MyoDrumRunning');
    end
end

function marker = obtainMarker(fig, provider, index, timeSeconds)
    marker = uint8(0);
    if ~isempty(provider)
        marker = uint8(provider(index, timeSeconds));
    elseif ~isempty(fig) && isgraphics(fig)
        marker = getappdata(fig, 'PendingMarker');
        setappdata(fig, 'PendingMarker', uint8(0));
    end
end

function value = localPercentile(values, percentage)
    values = sort(values(:));
    position = 1 + (numel(values) - 1) * percentage / 100;
    lower = floor(position);
    upper = ceil(position);
    fraction = position - lower;
    value = values(lower) * (1 - fraction) + values(upper) * fraction;
end
