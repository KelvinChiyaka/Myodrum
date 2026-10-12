function dashboard = createLiveSensorDashboard(cfg, options)
% CREATELIVESENSORDASHBOARD Create the low-rate IMU and EMG display.

    arguments
        cfg (1,1) struct
        options.Visible (1,1) string {mustBeMember(options.Visible, ...
            ["on", "off"])} = "on"
    end

    figureHandle = figure('Name', 'MyoDrum live sensors', ...
        'NumberTitle', 'off', 'Color', 'w', ...
        'Visible', char(options.Visible), ...
        'WindowKeyPressFcn', @keyPressed, ...
        'CloseRequestFcn', @closeRequested);
    setappdata(figureHandle, 'MyoDrumRunning', true);
    layout = tiledlayout(figureHandle, 2, 2, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    promptTitle = title(layout, 'LIVE PLAY', 'FontWeight', 'bold');

    accelAxes = nexttile(layout);
    accelLines = [line(accelAxes, nan, nan, 'Color', [0 0.45 0.74]), ...
        line(accelAxes, nan, nan, 'Color', [0.85 0.33 0.10])];
    configureAxes(accelAxes, 'Acceleration magnitude', '|a| (g)');
    legend(accelAxes, cfg.IMU.HandNames, 'Location', 'northwest');

    gyroAxes = nexttile(layout);
    gyroLines = [line(gyroAxes, nan, nan, 'Color', [0 0.45 0.74]), ...
        line(gyroAxes, nan, nan, 'Color', [0.85 0.33 0.10])];
    configureAxes(gyroAxes, 'Angular-rate magnitude', '|\omega| (rad/s)');
    legend(gyroAxes, cfg.IMU.HandNames, 'Location', 'northwest');

    emgAxes = nexttile(layout, [1 2]);
    colours = lines(numel(cfg.EMG.ChannelNames));
    emgLines = gobjects(1, numel(cfg.EMG.ChannelNames));
    for channel = 1:numel(emgLines)
        emgLines(channel) = line(emgAxes, nan, nan, ...
            'Color', colours(channel, :));
    end
    configureAxes(emgAxes, 'Normalised EMG activity', 'Normalised amplitude');
    legend(emgAxes, replace(cfg.EMG.ChannelNames, '_', ' '), ...
        'Location', 'northwest', 'NumColumns', 2);
    yline(emgAxes, cfg.Fusion.EMGThresholdNormalized, '--k', ...
        'Confirmation threshold');

    dashboard.Figure = figureHandle;
    dashboard.AccelAxes = accelAxes;
    dashboard.GyroAxes = gyroAxes;
    dashboard.EMGAxes = emgAxes;
    dashboard.AccelLines = accelLines;
    dashboard.GyroLines = gyroLines;
    dashboard.EMGLines = emgLines;
    dashboard.PromptTitle = promptTitle;
end

function configureAxes(axesHandle, plotTitle, yLabelText)
    grid(axesHandle, 'on');
    hold(axesHandle, 'on');
    xlabel(axesHandle, 'Recent time (s)');
    ylabel(axesHandle, yLabelText);
    title(axesHandle, plotTitle, 'Interpreter', 'none');
end

function keyPressed(figureHandle, event)
    if strcmpi(event.Key, 'q')
        setappdata(figureHandle, 'MyoDrumRunning', false);
    end
end

function closeRequested(figureHandle, ~)
    setappdata(figureHandle, 'MyoDrumRunning', false);
end
