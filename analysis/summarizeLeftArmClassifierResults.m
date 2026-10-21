function outputs = summarizeLeftArmClassifierResults(projectRoot, options)
% SUMMARIZELEFTARMCLASSIFIERRESULTS Build report-ready left-arm result tables.
% Candidate acceptance is intentionally kept separate from hit accuracy.
% Live accuracy remains undefined unless a ground-truth label is recorded for
% every attempted hit and no-hit movement.

    arguments
        projectRoot (1,1) string = string(fileparts(fileparts( ...
            mfilename('fullpath'))))
        options.RuleSessionId (1,1) string = "session_20261020_160445"
        options.MLSessionId (1,1) string = "session_20261020_163327"
        options.OutputFolder (1,1) string = ""
    end

    if options.OutputFolder == ""
        outputFolder = fullfile(projectRoot, 'data', 'processed', ...
            'results', 'leftThreeEMG');
    else
        outputFolder = options.OutputFolder;
    end
    if ~isfolder(outputFolder), mkdir(outputFolder); end

    sessionIds = [options.RuleSessionId; options.MLSessionId];
    expectedModes = ["rule-based"; "machine-learning"];
    summaryRows = repmat(emptySummaryRow, numel(sessionIds), 1);
    eventTables = cell(numel(sessionIds), 1);
    for index = 1:numel(sessionIds)
        [metadata, recording] = loadLiveSession( ...
            projectRoot, sessionIds(index), expectedModes(index));
        events = recording.fusionEvents;
        accepted = events.accepted;
        acceptedLatency_ms = 1000 * events.decisionLatency_s(accepted);

        row = emptySummaryRow;
        row.SessionId = sessionIds(index);
        row.Classifier = expectedModes(index);
        row.CompletedNormally = metadata.completedNormally;
        row.SampleCount = uint32(numel(recording.time_s));
        row.Duration_s = recording.time_s(end)-recording.time_s(1);
        row.StreamRate_Hz = metadata.streamStatistics.measuredRateHz;
        row.DroppedFrames = uint32(metadata.streamStatistics.droppedFrames);
        row.CRCFailures = uint32(metadata.streamStatistics.checksumFailures);
        row.CandidateEvents = uint32(height(events));
        row.AcceptedEvents = uint32(nnz(accepted));
        row.RejectedEvents = uint32(nnz(~accepted));
        row.CandidateAcceptance_pct = 100 * mean(accepted);
        row.AcceptedKick = uint32(nnz(accepted & ...
            events.orientationLabel == "KICK"));
        row.AcceptedSnare = uint32(nnz(accepted & ...
            events.orientationLabel == "SNARE"));
        row.AcceptedCrash = uint32(nnz(accepted & ...
            events.orientationLabel == "CRASH"));
        row.AcceptedDecisionLatencyMedian_ms = ...
            median(acceptedLatency_ms, 'omitnan');
        row.AcceptedDecisionLatencyP95_ms = ...
            prctile(acceptedLatency_ms, 95);
        row.AcceptedDecisionLatencyMaximum_ms = ...
            max(acceptedLatency_ms, [], 'omitnan');
        row.HostProcessingLagMedian_ms = 1000 * median( ...
            recording.hostProcessingLag_s, 'omitnan');
        row.HostProcessingLagP95_ms = 1000 * prctile( ...
            recording.hostProcessingLag_s, 95);
        row.HostProcessingLagMaximum_ms = 1000 * max( ...
            recording.hostProcessingLag_s, [], 'omitnan');
        row.GroundTruthAvailable = false;
        row.HitDetectionAccuracy_pct = NaN;
        row.DrumSelectionAccuracy_pct = NaN;
        row.FalsePositiveRate_pct = NaN;
        row.MissedHitRate_pct = NaN;
        row.Interpretation = "Usable for stream reliability, candidate " + ...
            "acceptance, rejection reasons and timing; not live accuracy.";
        summaryRows(index) = row;
        eventTables{index} = standardizeEvents( ...
            sessionIds(index), expectedModes(index), events);
    end

    classifierComparison = struct2table(summaryRows);
    eventLog = vertcat(eventTables{:});
    dataAudit = auditLeftArmSessions(projectRoot);
    modelValidation = readModelValidation(projectRoot);

    comparisonFile = fullfile(outputFolder, ...
        'leftArmClassifierComparison.csv');
    eventFile = fullfile(outputFolder, 'leftArmEventLog.csv');
    auditFile = fullfile(outputFolder, 'leftArmDataAudit.csv');
    validationFile = fullfile(outputFolder, ...
        'leftArmModelValidation.csv');
    matFile = fullfile(outputFolder, 'leftArmResults.mat');
    figureFile = fullfile(outputFolder, ...
        'leftArmClassifierComparison.png');
    notesFile = fullfile(outputFolder, 'README.txt');

    writetable(classifierComparison, comparisonFile);
    writetable(eventLog, eventFile);
    writetable(dataAudit, auditFile);
    writetable(modelValidation, validationFile);
    save(matFile, 'classifierComparison', 'eventLog', 'dataAudit', ...
        'modelValidation', '-v7.3');
    createComparisonFigure(classifierComparison, figureFile);
    writeNotes(notesFile);

    outputs = struct;
    outputs.outputFolder = string(outputFolder);
    outputs.classifierComparison = classifierComparison;
    outputs.eventLog = eventLog;
    outputs.dataAudit = dataAudit;
    outputs.modelValidation = modelValidation;
    outputs.figureFile = string(figureFile);

    fprintf('\nLeft-arm result summary created.\n');
    fprintf('Output folder:\n  %s\n', outputFolder);
    disp(classifierComparison(:, {'Classifier','CandidateEvents', ...
        'AcceptedEvents','RejectedEvents','CandidateAcceptance_pct', ...
        'AcceptedDecisionLatencyMedian_ms', ...
        'AcceptedDecisionLatencyP95_ms','HostProcessingLagMedian_ms', ...
        'HostProcessingLagP95_ms','GroundTruthAvailable'}));
end

function [metadata, recording] = loadLiveSession(root, sessionId, classifier)
    metadataFile = fullfile(root, 'data', 'metadata', 'sessions', ...
        sessionId, 'sessionMetadata.mat');
    rawFile = fullfile(root, 'data', 'raw', sessionId, ...
        'imuEmgStream.mat');
    if ~isfile(metadataFile) || ~isfile(rawFile)
        error('MyoDrum:MissingAnalysisSession', ...
            'Session %s is missing metadata or raw live data.', sessionId);
    end
    savedMetadata = load(metadataFile, 'metadata');
    metadata = savedMetadata.metadata;
    if ~isfield(metadata, 'completedNormally') || ...
            ~metadata.completedNormally || ...
            ~isfield(metadata, 'classifierMode') || ...
            string(metadata.classifierMode) ~= classifier
        error('MyoDrum:InvalidAnalysisSession', ...
            'Session %s is not a completed %s live recording.', ...
            sessionId, classifier);
    end
    recording = load(rawFile);
    required = {'fusionEvents','time_s','hostProcessingLag_s'};
    if ~all(isfield(recording, required))
        error('MyoDrum:InvalidAnalysisSession', ...
            'Session %s lacks required event or timing data.', sessionId);
    end
end

function result = standardizeEvents(sessionId, classifier, events)
    count = height(events);
    mlConfidence = nan(count,1);
    mlLabel = strings(count,1);
    ruleLabel = strings(count,1);
    if ismember('mlConfidence', events.Properties.VariableNames)
        mlConfidence = events.mlConfidence;
    end
    if ismember('mlLabel', events.Properties.VariableNames)
        mlLabel = string(events.mlLabel);
    end
    if ismember('ruleLabel', events.Properties.VariableNames)
        ruleLabel = string(events.ruleLabel);
    end
    result = table(repmat(sessionId,count,1), ...
        repmat(classifier,count,1), events.candidateId, ...
        events.handIndex, events.candidateTime_s, events.decisionTime_s, ...
        1000*events.decisionLatency_s, string(events.orientationLabel), ...
        mlLabel, ruleLabel, events.initialEMGScore, events.peakEMGScore, ...
        mlConfidence, events.accepted, string(events.reason), ...
        'VariableNames', {'SessionId','Classifier','CandidateId', ...
        'HandIndex','CandidateTime_s','DecisionTime_s', ...
        'DecisionLatency_ms','PredictedLabel','MLLabel','RuleLabel', ...
        'InitialEMGScore','PeakEMGScore','MLConfidence','Accepted', ...
        'Reason'});
end

function result = auditLeftArmSessions(root)
    files = dir(fullfile(root, 'data', 'metadata', 'sessions', ...
        'session_*', 'sessionMetadata.mat'));
    rows = repmat(emptyAuditRow, 0, 1);
    for index = 1:numel(files)
        saved = load(fullfile(files(index).folder, files(index).name), ...
            'metadata');
        metadata = saved.metadata;
        profile = metadataProfile(metadata);
        if profile ~= "left-three-emg", continue; end

        row = emptyAuditRow;
        row.SessionId = string(metadata.sessionId);
        row.Mode = getTextField(metadata, 'mode');
        row.Classifier = getTextField(metadata, 'classifierMode');
        row.CompletedNormally = getLogicalField( ...
            metadata, 'completedNormally');
        row.LoggedSamples = uint32(getNumericField( ...
            metadata, 'loggedSamples', 0));
        row.CompletedBlocks = uint16(getNumericField( ...
            metadata, 'completedBlocks', 0));
        row.HasBlockLabels = row.Mode == "ml-data-collection" && ...
            row.CompletedNormally && row.CompletedBlocks > 0;
        row.HasPerHitGroundTruth = false;
        rawFile = getTextField(metadata, 'rawDataFile');
        row.RawFileExists = rawFile ~= "" && isfile(rawFile);
        row.Usable = row.CompletedNormally && row.RawFileExists && ...
            ((row.Mode == "ml-data-collection" && ...
            row.CompletedBlocks == 12) || row.Mode == "imu-emg-stream");
        if row.Usable && row.Mode == "ml-data-collection"
            row.UsableFor = "ML training and grouped validation";
        elseif row.Usable && row.Mode == "imu-emg-stream"
            row.UsableFor = "Live behavior, reliability, rejection " + ...
                "reasons and latency";
        elseif ~row.CompletedNormally
            row.ExclusionReason = getTextField( ...
                metadata, 'failureIdentifier');
            if row.ExclusionReason == ""
                row.ExclusionReason = "INCOMPLETE_SESSION";
            end
        elseif ~row.RawFileExists
            row.ExclusionReason = "RAW_FILE_MISSING";
        else
            row.ExclusionReason = "INCOMPLETE_BLOCK_SET";
        end
        rows(end+1,1) = row; %#ok<AGROW>
    end
    result = struct2table(rows);
    if ~isempty(result), result = sortrows(result, 'SessionId'); end
end

function result = readModelValidation(root)
    modelFile = fullfile(root, 'data', 'models', ...
        'machineLearningLeft3', 'myoDrumLeft3BaggedTrees.mat');
    if ~isfile(modelFile)
        result = table("left-three-emg", "", NaN, NaN, uint32(0), ...
            uint32(0), 'VariableNames', {'SensorProfile','Method', ...
            'Accuracy','MacroF1','TrainingSessions','FeatureWindows'});
        return;
    end
    saved = load(modelFile, 'modelArtifact');
    artifact = saved.modelArtifact;
    result = table(string(artifact.sensorProfile), ...
        string(artifact.validationMethod), artifact.heldOutAccuracy, ...
        artifact.heldOutMacroF1, uint32(numel(artifact.trainingSessions)), ...
        uint32(artifact.trainingRows), 'VariableNames', ...
        {'SensorProfile','Method','Accuracy','MacroF1', ...
        'TrainingSessions','FeatureWindows'});
end

function createComparisonFigure(summary, outputFile)
    fig = figure('Visible','off','Color','w','Position',[100 100 1250 420]);
    cleanup = onCleanup(@()close(fig));
    names = categorical(summary.Classifier, summary.Classifier);
    tiledlayout(fig, 1, 3, 'Padding','compact','TileSpacing','compact');

    nexttile;
    bar(names, summary.CandidateAcceptance_pct);
    ylabel('Accepted candidates (%)'); ylim([0 100]); grid on;
    title('Acceptance, not accuracy');

    nexttile;
    bar(names, [summary.AcceptedDecisionLatencyMedian_ms, ...
        summary.AcceptedDecisionLatencyP95_ms]);
    ylabel('Classifier decision latency (ms)'); grid on;
    legend('Median','95th percentile','Location','northwest');
    title('Sensor-time decision latency');

    nexttile;
    bar(names, [summary.HostProcessingLagMedian_ms, ...
        summary.HostProcessingLagP95_ms]);
    ylabel('Host processing backlog (ms)'); grid on;
    legend('Median','95th percentile','Location','northwest');
    title('Recording-mode processing lag');
    exportgraphics(fig, outputFile, 'Resolution', 180);
end

function writeNotes(file)
    lines = [
        "MyoDrum left-arm result dataset"
        ""
        "leftArmDataAudit.csv lists every detected left-three-emg session and its usability."
        "leftArmClassifierComparison.csv compares the completed rule-based and ML live sessions."
        "leftArmEventLog.csv contains one row per classifier candidate for behavior analysis."
        "leftArmModelValidation.csv contains preliminary grouped ML validation results."
        ""
        "CandidateAcceptance_pct is NOT hit-detection accuracy."
        "The live sessions do not contain a ground-truth label for every attempted gesture."
        "Therefore accuracy, false-positive rate and missed-hit rate are deliberately NaN."
        "A labelled live evaluation or synchronized video annotation is required for those metrics."
        "Host processing lag is recording backlog, not classifier decision latency."
        ];
    writelines(lines, file);
end

function row = emptySummaryRow
    row = struct('SessionId',"",'Classifier',"", ...
        'CompletedNormally',false,'SampleCount',uint32(0), ...
        'Duration_s',NaN,'StreamRate_Hz',NaN, ...
        'DroppedFrames',uint32(0),'CRCFailures',uint32(0), ...
        'CandidateEvents',uint32(0),'AcceptedEvents',uint32(0), ...
        'RejectedEvents',uint32(0),'CandidateAcceptance_pct',NaN, ...
        'AcceptedKick',uint32(0),'AcceptedSnare',uint32(0), ...
        'AcceptedCrash',uint32(0), ...
        'AcceptedDecisionLatencyMedian_ms',NaN, ...
        'AcceptedDecisionLatencyP95_ms',NaN, ...
        'AcceptedDecisionLatencyMaximum_ms',NaN, ...
        'HostProcessingLagMedian_ms',NaN, ...
        'HostProcessingLagP95_ms',NaN, ...
        'HostProcessingLagMaximum_ms',NaN, ...
        'GroundTruthAvailable',false, ...
        'HitDetectionAccuracy_pct',NaN, ...
        'DrumSelectionAccuracy_pct',NaN, ...
        'FalsePositiveRate_pct',NaN,'MissedHitRate_pct',NaN, ...
        'Interpretation',"");
end

function row = emptyAuditRow
    row = struct('SessionId',"",'Mode',"",'Classifier',"", ...
        'CompletedNormally',false,'LoggedSamples',uint32(0), ...
        'CompletedBlocks',uint16(0),'HasBlockLabels',false, ...
        'HasPerHitGroundTruth',false,'RawFileExists',false, ...
        'Usable',false,'UsableFor',"",'ExclusionReason',"");
end

function value = metadataProfile(metadata)
    value = "";
    if isfield(metadata, 'sensorProfile')
        value = string(metadata.sensorProfile);
    elseif isfield(metadata, 'config') && ...
            isfield(metadata.config, 'Runtime') && ...
            isfield(metadata.config.Runtime, 'SensorProfile')
        value = string(metadata.config.Runtime.SensorProfile);
    end
end

function value = getTextField(source, field)
    value = "";
    if isfield(source, field), value = string(source.(field)); end
end

function value = getLogicalField(source, field)
    value = false;
    if isfield(source, field), value = logical(source.(field)); end
end

function value = getNumericField(source, field, defaultValue)
    value = defaultValue;
    if isfield(source, field), value = double(source.(field)); end
end
