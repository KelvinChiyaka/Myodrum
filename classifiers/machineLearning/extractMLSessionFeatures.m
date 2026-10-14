function [features, diagnostics, featureNames] = ...
        extractMLSessionFeatures(recorded, sessionId, cfg)
% EXTRACTMLSESSIONFEATURES Segment and describe one labelled collection.

    arguments
        recorded (1,1) struct
        sessionId (1,1) string
        cfg (1,1) struct
    end
    required = {'time_s','frameValid','accelCalibrated_g', ...
        'gyroCorrected_rad_s','emgNormalized','blockId','round', ...
        'handIndex','blockClassIndex','classNames','blockManifest'};
    if ~all(isfield(recorded, required))
        error('MyoDrum:InvalidMLFeatureSource', ...
            'The machine-learning recording schema is incomplete.');
    end
    manifest = recorded.blockManifest;
    if isempty(manifest) || ~all(manifest.Completed)
        error('MyoDrum:IncompleteMLFeatureSource', ...
            'Session %s contains incomplete collection blocks.', sessionId);
    end

    features = table;
    diagnosticRows = repmat(emptyDiagnostic, 0, 1);
    featureNames = strings(0,1);
    sampleId = uint32(0);
    for blockRow = 1:height(manifest)
        plan = manifest(blockRow,:);
        sourceRows = find(recorded.blockId == plan.BlockId);
        if isempty(sourceRows)
            error('MyoDrum:MissingMLBlockFrames', ...
                'Session %s block %d has no frames.', sessionId, plan.BlockId);
        end
        time = recorded.time_s(sourceRows);
        hand = double(plan.HandIndex);
        accel = reshape(recorded.accelCalibrated_g(sourceRows,:,hand), [], 3);
        gyro = reshape(recorded.gyroCorrected_rad_s(sourceRows,:,hand), [], 3);
        emg = recorded.emgNormalized(sourceRows,:);
        valid = recorded.frameValid(sourceRows,hand);
        dt = median(diff(time));
        if ~isfinite(dt) || dt <= 0
            error('MyoDrum:InvalidMLBlockTime', ...
                'Session %s block %d has invalid timestamps.', ...
                sessionId, plan.BlockId);
        end
        preSamples = round(cfg.MachineLearning.FeatureWindowPreSeconds / dt);
        postSamples = round(cfg.MachineLearning.FeatureWindowPostSeconds / dt);
        eligible = eligibleCenters(valid, preSamples, postSamples);
        score = movementScore(accel, gyro, dt);
        if plan.Label == "NO_HIT"
            requested = cfg.MachineLearning.NoHitWindowsPerBlock;
            centers = spreadCenters(eligible, requested);
            method = "EVEN_NO_HIT";
        else
            requested = double(plan.ExpectedHitCount);
            minimumSeparation = round( ...
                cfg.MachineLearning.FeatureMinimumSeparationSeconds / dt);
            centers = selectMovementPeaks(score, eligible, requested, ...
                minimumSeparation, max(1, round(0.04 / dt)));
            method = "IMU_PEAK";
        end
        if numel(centers) ~= requested
            error('MyoDrum:InsufficientMLFeatureWindows', ...
                ['Session %s block %d (%s %s) produced %d/%d valid ' ...
                 'windows. Inspect that block before training.'], ...
                sessionId, plan.BlockId, plan.Hand, plan.Label, ...
                numel(centers), requested);
        end

        for windowIndex = 1:numel(centers)
            center = centers(windowIndex);
            window = center-preSamples:center+postSamples;
            [values, names] = computeMLWindowFeatures( ...
                accel(window,:), gyro(window,:), emg(window,:), ...
                time(window), hand, cfg);
            if isempty(featureNames)
                featureNames = names(:);
            elseif ~isequal(featureNames, names(:))
                error('MyoDrum:MLFeatureDefinitionChanged', ...
                    'Feature names changed while processing session %s.', ...
                    sessionId);
            end
            sampleId = sampleId + 1;
            identity = table(sampleId, sessionId, ...
                sessionId + "_R" + string(plan.Round), plan.Round, ...
                plan.BlockId, plan.HandIndex, plan.Hand, plan.ClassIndex, ...
                plan.Label, uint8(windowIndex), time(window(1)), ...
                time(center), time(window(end)), score(center), method, ...
                'VariableNames', {'SampleId','SessionId', ...
                'SessionRoundGroup','Round','BlockId','HandIndex','Hand', ...
                'ClassIndex','Label','WindowIndex','WindowStart_s', ...
                'WindowCenter_s','WindowEnd_s','CandidateScore', ...
                'SelectionMethod'});
            featureTable = array2table(values, ...
                'VariableNames', cellstr(featureNames));
            features = [features; [identity, featureTable]]; %#ok<AGROW>
        end
        diagnostic = emptyDiagnostic;
        diagnostic.SessionId = sessionId;
        diagnostic.Round = plan.Round;
        diagnostic.BlockId = plan.BlockId;
        diagnostic.Hand = plan.Hand;
        diagnostic.Label = plan.Label;
        diagnostic.SourceFrames = uint32(numel(sourceRows));
        diagnostic.ValidFrames = uint32(sum(valid));
        diagnostic.ExtractedWindows = uint16(numel(centers));
        diagnostic.MinimumCandidateScore = min(score(centers));
        diagnostic.MedianCandidateScore = median(score(centers));
        diagnostic.SelectionMethod = method;
        diagnosticRows(end+1,1) = diagnostic; %#ok<AGROW>
    end
    diagnostics = struct2table(diagnosticRows);
end

function eligible = eligibleCenters(valid, preSamples, postSamples)
    count = numel(valid);
    eligible = false(count,1);
    for center = 1+preSamples:count-postSamples
        eligible(center) = all(valid(center-preSamples:center+postSamples));
    end
end

function centers = spreadCenters(eligible, requested)
    available = find(eligible);
    if numel(available) < requested
        centers = zeros(0,1);
        return;
    end
    positions = round(linspace(1, numel(available), requested+2));
    centers = available(positions(2:end-1));
    centers = unique(centers(:), 'stable');
end

function centers = selectMovementPeaks( ...
        score, eligible, requested, minimumSeparation, localRadius)
    localPeak = false(size(score));
    candidates = find(eligible);
    for index = candidates(:).'
        first = max(1, index-localRadius);
        last = min(numel(score), index+localRadius);
        localPeak(index) = score(index) >= max(score(first:last));
    end
    ranked = find(eligible & localPeak);
    [~, order] = sort(score(ranked), 'descend');
    ranked = ranked(order);
    centers = zeros(0,1);
    for index = ranked(:).'
        if isempty(centers) || all(abs(centers-index) >= minimumSeparation)
            centers(end+1,1) = index; %#ok<AGROW>
            if numel(centers) == requested, break; end
        end
    end
    if numel(centers) < requested
        remaining = find(eligible);
        [~, order] = sort(score(remaining), 'descend');
        remaining = remaining(order);
        for index = remaining(:).'
            if all(abs(centers-index) >= minimumSeparation)
                centers(end+1,1) = index; %#ok<AGROW>
                if numel(centers) == requested, break; end
            end
        end
    end
    centers = sort(centers);
end

function score = movementScore(accel, gyro, dt)
    accelMagnitude = vecnorm(accel, 2, 2);
    gyroMagnitude = vecnorm(gyro, 2, 2);
    jerk = [0; abs(diff(accelMagnitude)) / dt];
    score = 0.45 * robustPositive(abs(accelMagnitude-1)) + ...
        0.35 * robustPositive(gyroMagnitude) + ...
        0.20 * robustPositive(jerk);
    score = movmean(score, 3, 'Endpoints', 'shrink');
end

function result = robustPositive(values)
    centre = median(values);
    scale = 1.4826 * median(abs(values-centre));
    if ~isfinite(scale) || scale < 1e-9
        scale = max(std(values), 1e-9);
    end
    result = min(25, max(0, (values-centre)/scale));
end

function diagnostic = emptyDiagnostic
    diagnostic = struct('SessionId',"",'Round',uint8(0), ...
        'BlockId',uint16(0),'Hand',"",'Label',"", ...
        'SourceFrames',uint32(0),'ValidFrames',uint32(0), ...
        'ExtractedWindows',uint16(0),'MinimumCandidateScore',NaN, ...
        'MedianCandidateScore',NaN,'SelectionMethod',"");
end
