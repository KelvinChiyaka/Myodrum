function state = initializeLiveMLClassifier(projectRoot, cfg, options)
% INITIALIZELIVEMLCLASSIFIER Load and validate the saved live ML model.

    arguments
        projectRoot (1,1) string
        cfg (1,1) struct
        options.ModelFile (1,1) string = ""
        options.ClassifierMode (1,1) string = "machine-learning"
    end
    classifierMode = lower(strtrim(options.ClassifierMode));
    if ~ismember(classifierMode, ["machine-learning", "hybrid"])
        error('MyoDrum:InvalidLiveMLMode', ...
            'Live ML state supports machine-learning or hybrid mode.');
    end
    if options.ModelFile == ""
        modelFile = fullfile(projectRoot, 'data', 'models', ...
            cfg.MachineLearning.ModelFolderName, ...
            cfg.MachineLearning.ModelFileName);
    else
        modelFile = options.ModelFile;
    end
    if ~isfile(modelFile)
        error('MyoDrum:MissingLiveMLModel', ...
            ['Trained ML model is missing: %s. Run ' ...
             'trainMLBaggedTreeClassifier first.'], modelFile);
    end
    saved = load(modelFile, 'modelArtifact');
    if ~isfield(saved, 'modelArtifact')
        error('MyoDrum:InvalidLiveMLModel', ...
            'Model file does not contain modelArtifact.');
    end
    artifact = saved.modelArtifact;
    required = {'classifier','featureVersion','predictorNames', ...
        'classNames','modelVersion'};
    if ~all(isfield(artifact, required))
        error('MyoDrum:InvalidLiveMLModel', ...
            'Saved model metadata is incomplete.');
    end
    if string(artifact.featureVersion) ~= cfg.MachineLearning.FeatureVersion
        error('MyoDrum:LiveMLFeatureVersionMismatch', ...
            'Model feature version %s does not match runtime version %s.', ...
            artifact.featureVersion, cfg.MachineLearning.FeatureVersion);
    end
    if isfield(artifact, 'sensorProfile') && ...
            string(artifact.sensorProfile) ~= cfg.Runtime.SensorProfile
        error('MyoDrum:LiveMLSensorProfileMismatch', ...
            'Model sensor profile %s does not match runtime profile %s.', ...
            artifact.sensorProfile, cfg.Runtime.SensorProfile);
    end
    requiredClasses = cfg.MachineLearning.ClassNames(:);
    if ~all(ismember(requiredClasses, string(artifact.classNames(:))))
        error('MyoDrum:LiveMLClassMismatch', ...
            'Saved model does not contain every configured drum class.');
    end

    capacity = ceil(cfg.MachineLearning.LiveBufferSeconds * ...
        cfg.EMG.CombinedStreamRateHz);
    state.modelFile = string(modelFile);
    state.modelVersion = string(artifact.modelVersion);
    state.classifierMode = classifierMode;
    state.classifier = artifact.classifier;
    state.predictorNames = string(artifact.predictorNames(:));
    state.classNames = string(artifact.classNames(:));
    state.capacity = capacity;
    state.writeIndex = 0;
    state.sampleCount = 0;
    state.time_s = nan(capacity,1);
    state.accel_g = zeros(capacity,3,2);
    state.gyro_rad_s = zeros(capacity,3,2);
    state.emgNormalized = zeros(capacity,numel(cfg.EMG.ChannelNames));
    state.frameValid = false(capacity,2);
    state.emgValid = false(capacity,numel(cfg.EMG.ChannelNames));
    state.pendingActive = false(1,2);
    state.pendingTime_s = nan(1,2);
    state.pendingId = zeros(1,2,'uint32');
    state.pendingRuleLabel = strings(1,2);
    state.lastAcceptedTime_s = [-inf, -inf];
    state.nextCandidateId = uint32(1);
end
