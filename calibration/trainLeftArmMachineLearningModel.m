function [modelArtifact, validation] = trainLeftArmMachineLearningModel
% TRAINLEFTARMMACHINELEARNINGMODEL Extract and train the left-arm profile.
% With one complete session, validation holds out one complete collection
% round at a time. With three or more sessions it automatically holds out
% one complete session at a time.

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'classifiers', 'machineLearning'));
    extractMLFeatureDataset(string(root), ...
        SensorProfile="left-three-emg");
    [modelArtifact, validation] = trainMLBaggedTreeClassifier( ...
        string(root), SensorProfile="left-three-emg");
end
