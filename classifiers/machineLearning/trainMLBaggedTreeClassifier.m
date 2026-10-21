function [modelArtifact, validation] = ...
        trainMLBaggedTreeClassifier(projectRoot, options)
% TRAINMLBAGGEDTREECLASSIFIER Train and validate the first ML classifier.
% Every validation fold holds out one complete recording session. The final
% saved model is then fitted to all available sessions for later inference.

    arguments
        projectRoot (1,1) string = string(fileparts(fileparts( ...
            fileparts(mfilename('fullpath')))))
        options.FeatureFile (1,1) string = ""
        options.OutputFolder (1,1) string = ""
        options.SensorProfile (1,1) string = "four-channel"
    end

    addpath(fullfile(projectRoot, 'common'));
    cfg = myoDrumConfig(projectRoot, options.SensorProfile);
    if options.FeatureFile == ""
        featureFile = fullfile(cfg.Paths.ProcessedData, ...
            cfg.MachineLearning.DataFolderName, 'mlFeatureDataset.mat');
    else
        featureFile = options.FeatureFile;
    end
    if options.OutputFolder == ""
        outputFolder = fullfile(cfg.Paths.Models, ...
            cfg.MachineLearning.ModelFolderName);
    else
        outputFolder = options.OutputFolder;
    end
    if ~isfile(featureFile)
        error('MyoDrum:MissingMLFeatureDataset', ...
            'Feature dataset is missing: %s', featureFile);
    end

    saved = load(featureFile, 'features', 'featureMetadata');
    if ~isfield(saved, 'features') || ~isfield(saved, 'featureMetadata')
        error('MyoDrum:InvalidMLFeatureDataset', ...
            'Feature file must contain features and featureMetadata.');
    end
    features = saved.features;
    featureMetadata = saved.featureMetadata;
    requiredIdentity = ["SessionId", "SessionRoundGroup", "Label"];
    if ~all(ismember(requiredIdentity, string(features.Properties.VariableNames)))
        error('MyoDrum:InvalidMLFeatureDataset', ...
            'Feature table is missing session, round-group, or label data.');
    end

    predictorNames = string(featureMetadata.featureNames(:));
    classNames = string(featureMetadata.classNames(:));
    tableNames = string(features.Properties.VariableNames);
    if isempty(predictorNames) || ~all(ismember(predictorNames, tableNames))
        error('MyoDrum:InvalidMLFeatureDataset', ...
            'Feature metadata does not match the feature table.');
    end
    forbiddenPredictors = ["SampleId", "SessionId", ...
        "SessionRoundGroup", "Round", "BlockId", "HandIndex", "Hand", ...
        "ClassIndex", "Label", "WindowIndex", "WindowStart_s", ...
        "WindowCenter_s", "WindowEnd_s", "CandidateScore", ...
        "SelectionMethod"];
    leaked = intersect(predictorNames, forbiddenPredictors);
    if ~isempty(leaked)
        error('MyoDrum:MLIdentityLeakage', ...
            'Identity or label columns cannot be predictors: %s', ...
            strjoin(leaked, ', '));
    end

    predictors = features(:, cellstr(predictorNames));
    predictorMatrix = table2array(predictors);
    if ~isnumeric(predictorMatrix) || any(~isfinite(predictorMatrix), 'all')
        error('MyoDrum:InvalidMLPredictors', ...
            'All machine-learning predictors must be finite numeric values.');
    end
    response = categorical(string(features.Label), classNames);
    if any(isundefined(response))
        error('MyoDrum:UnknownMLClass', ...
            'The feature table contains a label not listed in metadata.');
    end
    sessions = unique(string(features.SessionId), 'stable');
    if numel(sessions) >= 3
        validationGroups = sessions;
        groupValues = string(features.SessionId);
        validationMethod = "leave-one-session-out";
    elseif cfg.Runtime.SensorProfile == "left-three-emg"
        validationGroups = unique(string(features.SessionRoundGroup), 'stable');
        groupValues = string(features.SessionRoundGroup);
        validationMethod = "leave-one-round-out-preliminary";
    else
        error('MyoDrum:InsufficientMLValidationSessions', ...
            ['At least three complete sessions are required for ' ...
             'leave-one-session-out validation.']);
    end
    if numel(validationGroups) < 3
        error('MyoDrum:InsufficientMLValidationGroups', ...
            'At least three independent validation groups are required.');
    end

    rng(42, 'twister');
    order = categorical(classNames, classNames);
    heldOutPrediction = categorical(strings(height(features),1), classNames);
    heldOutFold = zeros(height(features),1, 'uint8');
    foldRows = repmat(emptyFold, numel(validationGroups), 1);
    aggregateConfusion = zeros(numel(classNames));
    for fold = 1:numel(validationGroups)
        testRows = groupValues == validationGroups(fold);
        trainRows = ~testRows;
        if ~all(countcats(response(trainRows)) > 0) || ...
                ~all(countcats(response(testRows)) > 0)
            error('MyoDrum:IncompleteMLValidationFold', ...
                'Session %s does not support fixed-class validation.', ...
                validationGroups(fold));
        end
        classifier = fitBaggedTrees(predictors(trainRows,:), ...
            response(trainRows), cfg);
        predicted = categorical(string(predict(classifier, ...
            predictors(testRows,:))), classNames);
        heldOutPrediction(testRows) = predicted;
        heldOutFold(testRows) = uint8(fold);
        confusion = confusionmat(response(testRows), predicted, ...
            'Order', order);
        metrics = calculateClassificationMetrics(confusion, classNames);
        aggregateConfusion = aggregateConfusion + confusion;
        foldRows(fold).Fold = uint8(fold);
        foldRows(fold).HeldOutSession = validationGroups(fold);
        foldRows(fold).TrainingRows = uint32(sum(trainRows));
        foldRows(fold).TestRows = uint32(sum(testRows));
        foldRows(fold).Accuracy = metrics.accuracy;
        foldRows(fold).MacroPrecision = metrics.macroPrecision;
        foldRows(fold).MacroRecall = metrics.macroRecall;
        foldRows(fold).MacroF1 = metrics.macroF1;
    end

    overall = calculateClassificationMetrics(aggregateConfusion, classNames);
    sessionValidation = struct2table(foldRows);
    heldOutPredictions = table(features.SampleId, string(features.SessionId), ...
        string(features.SessionRoundGroup), string(features.Hand), ...
        string(features.Label), string(heldOutPrediction), heldOutFold, ...
        'VariableNames', {'SampleId','SessionId','SessionRoundGroup', ...
        'Hand','TrueLabel','PredictedLabel','Fold'});

    fullClassifier = fitBaggedTrees(predictors, response, cfg);
    importance = predictorImportance(fullClassifier);
    featureImportance = table(predictorNames, importance(:), ...
        'VariableNames', {'Feature','Importance'});
    featureImportance = sortrows(featureImportance, 'Importance', 'descend');

    validation.method = validationMethod;
    validation.sessions = sessions;
    validation.classNames = classNames;
    validation.confusionMatrix = aggregateConfusion;
    validation.overall = overall;
    validation.sessionSummary = sessionValidation;
    validation.heldOutPredictions = heldOutPredictions;
    validation.featureImportance = featureImportance;

    modelArtifact.modelVersion = cfg.MachineLearning.ModelVersion;
    modelArtifact.featureVersion = featureMetadata.featureVersion;
    modelArtifact.sensorProfile = cfg.Runtime.SensorProfile;
    modelArtifact.classifier = compact(fullClassifier);
    modelArtifact.predictorNames = predictorNames;
    modelArtifact.classNames = classNames;
    modelArtifact.trainingSessions = sessions;
    modelArtifact.trainingRows = uint32(height(features));
    modelArtifact.createdAt = string(datetime('now','TimeZone','local'));
    modelArtifact.validationMethod = validation.method;
    modelArtifact.heldOutAccuracy = overall.accuracy;
    modelArtifact.heldOutMacroF1 = overall.macroF1;

    if ~isfolder(outputFolder), mkdir(outputFolder); end
    modelFile = fullfile(outputFolder, ...
        cfg.MachineLearning.ModelFileName);
    save(modelFile, 'modelArtifact', 'validation', '-v7.3');
    writetable(sessionValidation, ...
        fullfile(outputFolder, 'sessionValidation.csv'));
    writetable(overall.perClass, ...
        fullfile(outputFolder, 'classMetrics.csv'));
    writetable(heldOutPredictions, ...
        fullfile(outputFolder, 'heldOutPredictions.csv'));
    writetable(featureImportance, ...
        fullfile(outputFolder, 'featureImportance.csv'));
    writeConfusionTable(aggregateConfusion, classNames, ...
        fullfile(outputFolder, 'confusionMatrix.csv'));
    createValidationFigure(aggregateConfusion, classNames, ...
        sessionValidation, fullfile(outputFolder, 'validationSummary.png'));

    fprintf('\nMyoDrum bagged-tree training complete.\n');
    fprintf('Validation: %s.\n', validation.method);
    fprintf('Held-out accuracy: %.1f%%\n', 100*overall.accuracy);
    fprintf('Held-out macro F1: %.3f\n', overall.macroF1);
    disp(sessionValidation(:, {'HeldOutSession','TestRows','Accuracy','MacroF1'}));
    disp(overall.perClass);
    fprintf('Saved model:\n  %s\n', modelFile);
end

function classifier = fitBaggedTrees(predictors, response, cfg)
    learner = templateTree( ...
        'MinLeafSize', cfg.MachineLearning.BaggedTreeMinLeafSize, ...
        'MaxNumSplits', cfg.MachineLearning.BaggedTreeMaxSplits);
    classifier = fitcensemble(predictors, response, ...
        'Method', 'Bag', ...
        'NumLearningCycles', cfg.MachineLearning.BaggedTreeCount, ...
        'Learners', learner, ...
        'ClassNames', categories(response));
end

function row = emptyFold
    row = struct('Fold', uint8(0), 'HeldOutSession', "", ...
        'TrainingRows', uint32(0), 'TestRows', uint32(0), ...
        'Accuracy', NaN, 'MacroPrecision', NaN, 'MacroRecall', NaN, ...
        'MacroF1', NaN);
end

function writeConfusionTable(confusion, classNames, outputFile)
    variableNames = "Predicted_" + matlab.lang.makeValidName(classNames);
    result = array2table(confusion, 'VariableNames', cellstr(variableNames));
    result = addvars(result, classNames, 'Before', 1, ...
        'NewVariableNames', 'TrueClass');
    writetable(result, outputFile);
end

function createValidationFigure(confusion, classNames, sessions, outputFile)
    figureHandle = figure('Visible','off','Color','w', ...
        'Position',[100 100 1200 480]);
    cleanup = onCleanup(@()close(figureHandle)); %#ok<NASGU>
    tiledlayout(figureHandle, 1, 2, 'Padding','compact', ...
        'TileSpacing','compact');

    nexttile;
    percentage = 100 .* confusion ./ max(sum(confusion,2), 1);
    imagesc(percentage, [0 100]);
    axis square;
    colorbar;
    colormap(parula);
    xticks(1:numel(classNames));
    yticks(1:numel(classNames));
    xticklabels(classNames);
    yticklabels(classNames);
    xlabel('Predicted class');
    ylabel('True class');
    title('Held-out confusion matrix (%)');
    for row = 1:numel(classNames)
        for column = 1:numel(classNames)
            text(column, row, sprintf('%.1f', percentage(row,column)), ...
                'HorizontalAlignment','center','FontWeight','bold');
        end
    end

    nexttile;
    sessionNumber = 1:height(sessions);
    bar(sessionNumber, 100*sessions.Accuracy, ...
        'FaceColor',[0.12 0.45 0.75]);
    xticks(sessionNumber);
    xticklabels("Session " + string(sessionNumber));
    ylim([0 100]);
    ylabel('Accuracy (%)');
    xlabel('Completely unseen session');
    title('Leave-one-session-out accuracy');
    grid on;
    exportgraphics(figureHandle, outputFile, 'Resolution', 180);
end
