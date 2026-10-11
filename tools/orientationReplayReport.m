function orientationReplayReport(sessionIds, params)
% ORIENTATIONREPLAYREPORT Compare legacy and new orientation during stillness.
%
%   orientationReplayReport(["session_20261010_145313", ...])
%
% For every quiet interval (gyro norm below 3 deg/s for at least 1.5 s,
% skipping the first 5 samples of each run) it reports how fast the logged
% legacy orientation and the new estimator keep moving. A still stick should
% give zero. It also reports convergence and the spread of the new features
% at the moment each hit's downstroke armed.

    arguments
        sessionIds (1,:) string
        params struct = struct
    end

    for sessionId = sessionIds
        r = replayOrientation(sessionId, params);
        fprintf('\n=== %s ===\n', sessionId);
        fprintf('detector replay matches logged hits: %d\n', r.hitsMatchLog);
        for hand = 1:2
            runs = quietRuns(r.gyroRadPerSec(:, :, hand), r.time);
            if isempty(runs)
                fprintf(' hand %d: no quiet runs of at least 1.5 s\n', hand);
                continue;
            end
            legacy = runSlew(r.legacyYawPitchRoll(:, :, hand), r.time, runs, ...
                [true false false]);
            legacyTilt = runSlew(r.legacyYawPitchRoll(:, :, hand), r.time, ...
                runs, [false true true]);
            newAz = runSlew(r.features(:, :, hand), r.time, runs, ...
                [true false false]);
            newTilt = runSlew(r.features(:, :, hand), r.time, runs, ...
                [false true true]);
            fprintf([' hand %d: %d quiet runs | azimuth slew deg/s: legacy ' ...
                '%.2f -> new %.2f | tilt slew deg/s: legacy %.2f -> new %.2f\n'], ...
                hand, size(runs, 1), legacy, newAz, legacyTilt, newTilt);
            firstConverged = find(r.converged(:, hand), 1);
            if isempty(firstConverged)
                fprintf('         never reported converged\n');
            else
                fprintf('         first converged at t = %.1f s\n', ...
                    r.time(firstConverged));
            end
        end
    end
end

function runs = quietRuns(gyroRadPerSec, time)
    quiet = vecnorm(rad2deg(gyroRadPerSec), 2, 2) < 3;
    edges = diff([0; quiet(:); 0]);
    first = find(edges == 1);
    last = find(edges == -1) - 1;
    runs = zeros(0, 2);
    for runIndex = 1:numel(first)
        a = first(runIndex) + 5;
        b = last(runIndex);
        if b > a && time(b) - time(a) >= 1.5
            runs(end + 1, :) = [a, b]; %#ok<AGROW>
        end
    end
end

function slew = runSlew(angles, time, runs, columns)
    values = zeros(size(runs, 1), 1);
    for runIndex = 1:size(runs, 1)
        a = runs(runIndex, 1);
        b = runs(runIndex, 2);
        change = zeros(1, sum(columns));
        columnIndices = find(columns);
        for k = 1:numel(columnIndices)
            column = angles(a:b, columnIndices(k));
            column = rad2deg(unwrap(deg2rad(column)));
            change(k) = abs(column(end) - column(1)) / (time(b) - time(a));
        end
        values(runIndex) = max(change);
    end
    slew = mean(values, 'omitnan');
end
