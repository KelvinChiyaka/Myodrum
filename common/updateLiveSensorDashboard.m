function updateLiveSensorDashboard( ...
        dashboard, log, cfg, labels, emgScores, prompt)
% UPDATELIVESENSORDASHBOARD Refresh a decimated rolling sensor view.

    arguments
        dashboard (1,1) struct
        log (1,1) struct
        cfg (1,1) struct
        labels (1,2) string = ["", ""]
        emgScores (1,2) double = [0, 0]
        prompt (1,1) string = ""
    end
    if isempty(log.time_s) || ~isgraphics(dashboard.Figure)
        return;
    end

    finalIndex = numel(log.time_s);
    firstIndex = find(log.time_s >= ...
        log.time_s(end) - cfg.Visualization.WindowSeconds, 1, 'first');
    indices = firstIndex:finalIndex;
    if numel(indices) > cfg.Visualization.MaximumPoints
        indices = round(linspace(firstIndex, finalIndex, ...
            cfg.Visualization.MaximumPoints));
    end
    time = log.time_s(indices) - log.time_s(end);

    accel = sqrt(sum(log.accelCalibrated_g(indices, :, :) .^ 2, 2));
    gyro = sqrt(sum(log.gyroCorrected_rad_s(indices, :, :) .^ 2, 2));
    for hand = 1:2
        set(dashboard.AccelLines(hand), 'XData', time, ...
            'YData', reshape(accel(:, 1, hand), [], 1));
        set(dashboard.GyroLines(hand), 'XData', time, ...
            'YData', reshape(gyro(:, 1, hand), [], 1));
    end
    for channel = 1:numel(dashboard.EMGLines)
        set(dashboard.EMGLines(channel), 'XData', time, ...
            'YData', log.emgNormalized(indices, channel));
    end
    xLimits = [-cfg.Visualization.WindowSeconds, 0];
    xlim(dashboard.AccelAxes, xLimits);
    xlim(dashboard.GyroAxes, xLimits);
    xlim(dashboard.EMGAxes, xLimits);
    title(dashboard.AccelAxes, sprintf( ...
        'IMU | LEFT: %s   RIGHT: %s', displayLabel(labels(1)), ...
        displayLabel(labels(2))), 'Interpreter', 'none');
    title(dashboard.EMGAxes, sprintf( ...
        'Normalised EMG | LEFT %.2f   RIGHT %.2f', ...
        emgScores(1), emgScores(2)), 'Interpreter', 'none');
    if prompt ~= ""
        dashboard.Figure.Name = "MyoDrum | " + prompt;
        dashboard.PromptTitle.String = prompt;
    else
        dashboard.Figure.Name = 'MyoDrum live sensors';
        dashboard.PromptTitle.String = 'LIVE PLAY';
    end
    drawnow limitrate;
end

function result = displayLabel(label)
    if label == ""
        result = 'UNCERTAIN';
    else
        result = char(label);
    end
end
