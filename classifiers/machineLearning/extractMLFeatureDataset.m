function [features, featureMetadata, blockDiagnostics] = ...
        extractMLFeatureDataset(projectRoot, options)
% EXTRACTMLFEATUREDATASET Build one grouped feature table from ML sessions.

    arguments
        projectRoot (1,1) string = string(fileparts(fileparts( ...
            fileparts(mfilename('fullpath')))))
        options.SessionIds (:,1) string = strings(0,1)
        options.OutputFolder (1,1) string = ""
        options.SensorProfile (1,1) string = "four-channel"
    end
    addpath(fullfile(projectRoot, 'common'));
    cfg = myoDrumConfig(projectRoot, options.SensorProfile);
    if options.OutputFolder == ""
        outputFolder = fullfile(cfg.Paths.ProcessedData, ...
            cfg.MachineLearning.DataFolderName);
    else
        outputFolder = options.OutputFolder;
    end
    sessionIds = options.SessionIds;
    if isempty(sessionIds)
        sessionIds = discoverCompleteSessions(cfg);
    end
    if isempty(sessionIds)
        error('MyoDrum:NoCompleteMLSessions', ...
            'No completed machine-learning collection sessions were found.');
    end
    if numel(unique(sessionIds)) ~= numel(sessionIds)
        error('MyoDrum:DuplicateMLSession', ...
            'Each source session must be listed only once.');
    end

    features = table;
    blockDiagnostics = table;
    featureNames = strings(0,1);
    classNames = strings(0,1);
    for sessionIndex = 1:numel(sessionIds)
        sessionId = sessionIds(sessionIndex);
        rawFile = fullfile(cfg.Paths.RawData, sessionId, ...
            'mlTrainingStream.mat');
        requireFile(rawFile);
        recorded = load(rawFile, 'time_s', 'frameValid', ...
            'accelCalibrated_g', 'gyroCorrected_rad_s', ...
            'emgNormalized', 'blockId', 'round', 'handIndex', ...
            'blockClassIndex', 'classNames', 'blockManifest');
        [sessionFeatures, sessionDiagnostics, currentNames] = ...
            extractMLSessionFeatures(recorded, sessionId, cfg);
        if isempty(featureNames)
            featureNames = currentNames;
            classNames = recorded.classNames(:);
        elseif ~isequal(featureNames, currentNames) || ...
                ~isequal(classNames, recorded.classNames(:))
            error('MyoDrum:IncompatibleMLSessions', ...
                'Feature or class definitions differ in session %s.', ...
                sessionId);
        end
        features = [features; sessionFeatures]; %#ok<AGROW>
        blockDiagnostics = [blockDiagnostics; sessionDiagnostics]; %#ok<AGROW>

        sessionFolder = fullfile(outputFolder, sessionId);
        if ~isfolder(sessionFolder), mkdir(sessionFolder); end
        save(fullfile(sessionFolder, 'mlFeatures.mat'), ...
            'sessionFeatures', 'sessionDiagnostics', 'currentNames');
        writetable(sessionFeatures, ...
            fullfile(sessionFolder, 'mlFeatures.csv'));
    end

    featureMetadata.createdAt = string(datetime('now','TimeZone','local'));
    featureMetadata.featureVersion = cfg.MachineLearning.FeatureVersion;
    featureMetadata.sensorProfile = cfg.Runtime.SensorProfile;
    featureMetadata.sourceSessions = sessionIds;
    featureMetadata.featureNames = featureNames;
    featureMetadata.classNames = classNames;
    featureMetadata.groupVariables = ["SessionId", "SessionRoundGroup"];
    featureMetadata.windowPreSeconds = ...
        cfg.MachineLearning.FeatureWindowPreSeconds;
    featureMetadata.windowPostSeconds = ...
        cfg.MachineLearning.FeatureWindowPostSeconds;
    featureMetadata.minimumSeparationSeconds = ...
        cfg.MachineLearning.FeatureMinimumSeparationSeconds;
    featureMetadata.labelMeaning = [ ...
        "Each row is one fixed motion window. Labels come from the ", ...
        "operator-prompted block; session and round columns are grouping ", ...
        "variables and must not be model predictors."];

    if ~isfolder(outputFolder), mkdir(outputFolder); end
    outputMat = fullfile(outputFolder, 'mlFeatureDataset.mat');
    outputCsv = fullfile(outputFolder, 'mlFeatureDataset.csv');
    diagnosticCsv = fullfile(outputFolder, 'mlFeatureDiagnostics.csv');
    save(outputMat, 'features', 'featureMetadata', 'blockDiagnostics', '-v7.3');
    writetable(features, outputCsv);
    writetable(blockDiagnostics, diagnosticCsv);

    fprintf('\nMyoDrum ML feature extraction complete.\n');
    fprintf('Sessions: %d\nFeature windows: %d\nFeatures per window: %d\n', ...
        numel(sessionIds), height(features), numel(featureNames));
    for classIndex = 1:numel(classNames)
        fprintf('  %-7s %d\n', classNames(classIndex), ...
            sum(features.Label == classNames(classIndex)));
    end
    fprintf('Feature MAT:\n  %s\nFeature CSV:\n  %s\n', ...
        outputMat, outputCsv);
end

function sessionIds = discoverCompleteSessions(cfg)
    folders = dir(fullfile(cfg.Paths.RawData, 'session_*'));
    sessionIds = strings(0,1);
    for folderIndex = 1:numel(folders)
        if ~folders(folderIndex).isdir, continue; end
        sessionId = string(folders(folderIndex).name);
        rawFile = fullfile(folders(folderIndex).folder, sessionId, ...
            'mlTrainingStream.mat');
        metadataFile = fullfile(cfg.Paths.Metadata, 'sessions', ...
            sessionId, 'sessionMetadata.mat');
        if ~isfile(rawFile) || ~isfile(metadataFile), continue; end
        saved = load(metadataFile, 'metadata');
        metadata = saved.metadata;
        profile = metadataProfile(metadata);
        if isfield(metadata, 'mode') && metadata.mode == "ml-data-collection" && ...
                isfield(metadata, 'completedNormally') && ...
                metadata.completedNormally && ...
                profile == cfg.Runtime.SensorProfile
            sessionIds(end+1,1) = sessionId; %#ok<AGROW>
        end
    end
    sessionIds = sort(sessionIds);
end

function profile = metadataProfile(metadata)
    if isfield(metadata, 'sensorProfile')
        profile = string(metadata.sensorProfile);
    elseif isfield(metadata, 'config') && ...
            isfield(metadata.config, 'Runtime') && ...
            isfield(metadata.config.Runtime, 'SensorProfile')
        profile = string(metadata.config.Runtime.SensorProfile);
    else
        profile = "four-channel";
    end
end

function requireFile(file)
    if ~isfile(file)
        error('MyoDrum:MissingMLTrainingStream', ...
            'Machine-learning stream is missing: %s', file);
    end
end
