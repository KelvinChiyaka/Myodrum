function tests = testRecordedSessionReplay
% TESTRECORDEDSESSIONREPLAY Verify detector replay against logged hit flags.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'classifiers', 'ruleBased'));
    addpath(fullfile(root, 'tests', 'helpers'));
    testCase.TestData.Root = root;
end

function testAllAvailableRecordedSessions(testCase)
    root = testCase.TestData.Root;
    sessions = dir(fullfile(root, 'data', 'raw', ...
        'session_*', 'dualKickIMU.mat'));
    assumeTrue(testCase, ~isempty(sessions), ...
        'No local dual-IMU recordings are available; replay was skipped.');

    currentKickCfg = myoDrumConfig(root).Kick;
    for sessionIndex = 1:numel(sessions)
        rawFile = string(fullfile( ...
            sessions(sessionIndex).folder, sessions(sessionIndex).name));
        [~, sessionFolderName] = fileparts(sessions(sessionIndex).folder);
        sessionId = string(sessionFolderName);
        % These two sessions were recorded with earlier detector code
        % (guard time 0.08 s, a fast re-arm rule that no longer exists), so
        % a mismatch there is expected and says nothing about regressions.
        if any(sessionId == ["session_20261010_130411", ...
                "session_20261010_131918"])
            log(testCase, 1, sprintf( ...
                'Skipped %s: recorded with superseded detector code.', ...
                sessionId));
            continue;
        end
        metadataFolder = fullfile(root, 'data', 'metadata', ...
            'sessions', sessionId);
        metadataFile = fullfile(metadataFolder, 'sessionMetadata.mat');
        calibrationFile = fullfile(metadataFolder, 'imuSessionCalibration.mat');
        assertTrue(testCase, isfile(metadataFile), ...
            sprintf('Missing replay metadata: %s', metadataFile));
        assertTrue(testCase, isfile(calibrationFile), ...
            sprintf('Missing replay calibration: %s', calibrationFile));

        session = load(metadataFile, 'metadata');
        savedCalibration = load(calibrationFile, 'calibration');
        kickCfg = currentKickCfg;
        if isfield(session.metadata, 'config') && ...
                isfield(session.metadata.config, 'Kick')
            kickCfg = overlayFields(kickCfg, session.metadata.config.Kick);
        end
        sampleRateHz = session.metadata.measuredSampleRateHz;
        [~, replayedHits] = replayDetector(rawFile, ...
            savedCalibration.calibration.directions, kickCfg, sampleRateHz);
        recorded = load(rawFile, 'kickHit');

        difference = xor(replayedHits, logical(recorded.kickHit));
        if any(difference, 'all')
            detail = formatDifferences(difference, replayedHits, ...
                logical(recorded.kickHit));
            verifyFail(testCase, sprintf( ...
                'Replay differs for %s. %s', sessionId, detail));
        end
    end
end

function merged = overlayFields(defaults, saved)
    merged = defaults;
    names = fieldnames(saved);
    for fieldIndex = 1:numel(names)
        merged.(names{fieldIndex}) = saved.(names{fieldIndex});
    end
end

function detail = formatDifferences(difference, replayed, recorded)
    messages = strings(0, 1);
    for handIndex = 1:2
        indices = find(difference(:, handIndex));
        if isempty(indices)
            continue;
        end
        indexText = strjoin(string(indices.'), ',');
        messages(end + 1) = sprintf( ...
            ['hand %d: %d differences, first sample %d, all samples [%s], ' ...
             'recorded hits %d, replayed hits %d'], ...
            handIndex, numel(indices), indices(1), indexText, ...
            sum(recorded(:, handIndex)), sum(replayed(:, handIndex))); %#ok<AGROW>
    end
    detail = strjoin(messages, '; ');
end
