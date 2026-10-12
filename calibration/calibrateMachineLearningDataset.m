function calibrateMachineLearningDataset
% CALIBRATEMACHINELEARNINGDATASET Record labelled IMU and EMG training data.
% This calibration workflow is deliberately separate from mainMyoDrum,
% which is reserved for live instrument operation.

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'classifiers', 'machineLearning'));
    addpath(fullfile(root, 'drumRunTests', 'machineLearning'));
    mainMLDataCollection;
end
