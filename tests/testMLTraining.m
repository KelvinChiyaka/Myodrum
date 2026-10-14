function tests = testMLTraining
% TESTMLTRAINING Verify leakage guards and metric calculations.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'classifiers','machineLearning'));
    testCase.TestData.Root = root;
end

function testClassificationMetrics(testCase)
    confusion = [8 2;1 9];
    metrics = calculateClassificationMetrics(confusion,["A";"B"]);
    verifyEqual(testCase,metrics.accuracy,0.85,'AbsTol',1e-12);
    verifyEqual(testCase,metrics.perClass.Recall,[0.8;0.9], ...
        'AbsTol',1e-12);
    verifyEqual(testCase,metrics.perClass.Precision,[8/9;9/11], ...
        'AbsTol',1e-12);
end

function testTrainingSourceUsesSessionHeldOutValidation(testCase)
    source = fileread(fullfile(testCase.TestData.Root,'classifiers', ...
        'machineLearning','trainMLBaggedTreeClassifier.m'));
    verifyNotEmpty(testCase,regexp(source, ...
        'testRows = features\.SessionId == sessions\(fold\)', 'once'));
    verifyNotEmpty(testCase,regexp(source,'fitcensemble', 'once'));
    verifyNotEmpty(testCase,regexp(source,'MyoDrum:MLIdentityLeakage','once'));
    verifyNotEmpty(testCase,regexp(source,'predictorNames', 'once'));
end
