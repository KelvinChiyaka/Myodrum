function results = analyzeDrumMapData(csvFile)
% ANALYZEDRUMMAPDATA Compare classifiers using held-out collection rounds.

    if nargin < 1 || strlength(string(csvFile)) == 0
        root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        csvFile = fullfile(root, 'data', 'raw', 'twoIMUDrumHits.csv');
    end
    if ~isfile(csvFile)
        error('MyoDrum:MissingDataset', ...
            ['Dataset not found: %s\nPass the collected CSV path explicitly, ' ...
             'or place it in data/raw/.'], csvFile);
    end
    data = readtable(csvFile, 'TextType', 'string');
    featureNames = startsWith(data.Properties.VariableNames, 'F');
    drumOrder = ["CRASH", "HIHAT", "TOM", "KICK", "SNARE"];
    handOrder = ["LEFT", "RIGHT"];
    methods = ["Centroid", "KNN3", "KNN7", "LDA", "LinearSVM", "BaggedTrees"];

    results = struct;
    for handIndex = 1:numel(handOrder)
        rows = data.Hand == handOrder(handIndex);
        x = data{rows, featureNames};
        y = data.Drum(rows);
        rounds = data.Round(rows);

        randomPredictions = strings(size(y, 1), numel(methods));
        roundMethodPredictions = strings(size(y, 1), numel(methods));

        for fold = 1:max(rounds)
            train = rounds ~= fold;
            test = rounds == fold;
            [xTrain, xTest] = standardizeTrainTest(x(train, :), x(test, :));
            for methodIndex = 1:numel(methods)
                roundMethodPredictions(test, methodIndex) = classifyModel( ...
                    xTrain, y(train), xTest, methods(methodIndex), drumOrder);
            end
        end
        roundPredictions = roundMethodPredictions(:, 1);

        rng(42);
        randomFold = mod(randperm(size(x, 1))' - 1, 5) + 1;
        for fold = 1:5
            train = randomFold ~= fold;
            test = randomFold == fold;
            [xTrain, xTest] = standardizeTrainTest(x(train, :), x(test, :));
            for methodIndex = 1:numel(methods)
                randomPredictions(test, methodIndex) = classifyModel( ...
                    xTrain, y(train), xTest, methods(methodIndex), drumOrder);
            end
        end

        roundAccuracy = mean(roundMethodPredictions == y, 1);
        randomAccuracy = mean(randomPredictions == y, 1);
        [bestAccuracy, bestIndex] = max(roundAccuracy);
        confusion = confusionmat(y, roundMethodPredictions(:, bestIndex), ...
            'Order', drumOrder);

        fprintf('\n%s HAND\n', handOrder(handIndex));
        fprintf('%-14s  Round-held-out  Random-fold\n', 'Method');
        for methodIndex = 1:numel(methods)
            fprintf('%-14s      %5.1f%%        %5.1f%%\n', ...
                methods(methodIndex), 100*roundAccuracy(methodIndex), ...
                100*randomAccuracy(methodIndex));
        end
        fprintf('Best round-held-out method: %s (%.1f%%)\n', ...
            methods(bestIndex), 100*bestAccuracy);
        fprintf('Rows=actual, columns=predicted [CRASH HIHAT TOM KICK SNARE]:\n');
        disp(confusion);

        results(handIndex).hand = handOrder(handIndex);
        results(handIndex).methods = methods;
        results(handIndex).roundAccuracy = roundAccuracy;
        results(handIndex).randomAccuracy = randomAccuracy;
        results(handIndex).bestMethod = methods(bestIndex);
        results(handIndex).bestAccuracy = bestAccuracy;
        results(handIndex).confusion = confusion;
        results(handIndex).centroidConfusion = confusionmat( ...
            y, roundPredictions, 'Order', drumOrder);
    end
end

function [trainZ, testZ] = standardizeTrainTest(trainX, testX)
    mu = mean(trainX, 1);
    sigma = std(trainX, 0, 1);
    sigma(sigma < 1e-9) = 1;
    trainZ = (trainX - mu) ./ sigma;
    testZ = (testX - mu) ./ sigma;
end

function predicted = classifyModel(trainX, trainY, testX, method, drumOrder)
    switch method
        case "Centroid"
            centroids = zeros(numel(drumOrder), size(trainX, 2));
            for k = 1:numel(drumOrder)
                centroids(k, :) = mean(trainX(trainY == drumOrder(k), :), 1);
            end
            predicted = strings(size(testX, 1), 1);
            for row = 1:size(testX, 1)
                distance = sum((centroids - testX(row, :)).^2, 2);
                [~, index] = min(distance);
                predicted(row) = drumOrder(index);
            end
        case "KNN3"
            model = fitcknn(trainX, trainY, 'NumNeighbors', 3, ...
                'Distance', 'euclidean', 'DistanceWeight', 'squaredinverse');
            predicted = string(predict(model, testX));
        case "KNN7"
            model = fitcknn(trainX, trainY, 'NumNeighbors', 7, ...
                'Distance', 'euclidean', 'DistanceWeight', 'squaredinverse');
            predicted = string(predict(model, testX));
        case "LDA"
            model = fitcdiscr(trainX, trainY, 'DiscrimType', 'pseudolinear');
            predicted = string(predict(model, testX));
        case "LinearSVM"
            learner = templateSVM('KernelFunction', 'linear', ...
                'BoxConstraint', 1, 'Standardize', false);
            model = fitcecoc(trainX, trainY, 'Learners', learner, ...
                'Coding', 'onevsone');
            predicted = string(predict(model, testX));
        case "BaggedTrees"
            tree = templateTree('MaxNumSplits', 30, 'MinLeafSize', 5);
            model = fitcensemble(trainX, trainY, 'Method', 'Bag', ...
                'Learners', tree, 'NumLearningCycles', 100);
            predicted = string(predict(model, testX));
    end
end
