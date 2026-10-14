function metrics = calculateClassificationMetrics(confusion, classNames)
% CALCULATECLASSIFICATIONMETRICS Summarize a fixed-order confusion matrix.

    arguments
        confusion (:,:) double {mustBeNonnegative}
        classNames (:,1) string
    end
    if size(confusion,1) ~= size(confusion,2) || ...
            size(confusion,1) ~= numel(classNames)
        error('MyoDrum:InvalidConfusionMatrix', ...
            'Confusion matrix size must match the number of classes.');
    end
    trueCount = sum(confusion,2);
    predictedCount = sum(confusion,1).';
    correct = diag(confusion);
    recall = correct ./ max(trueCount,1);
    precision = correct ./ max(predictedCount,1);
    f1 = 2 .* precision .* recall ./ max(precision+recall,eps);
    metrics.accuracy = sum(correct) / max(sum(confusion,'all'),1);
    metrics.macroPrecision = mean(precision);
    metrics.macroRecall = mean(recall);
    metrics.macroF1 = mean(f1);
    metrics.perClass = table(classNames, trueCount, predictedCount, ...
        precision, recall, f1, 'VariableNames', {'Class','TrueCount', ...
        'PredictedCount','Precision','Recall','F1'});
end
