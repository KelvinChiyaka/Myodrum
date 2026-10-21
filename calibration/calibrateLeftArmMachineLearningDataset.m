function calibrateLeftArmMachineLearningDataset
% CALIBRATELEFTARMMACHINELEARNINGDATASET Record the left three-EMG profile.
% Three bipolar channels use six sensing electrodes. The synchronized packet
% still contains both IMUs and four ADC slots, but only left IMU 0x68 and
% A0/A1/A2 are required; A4 and the right-hand signals are ignored.

    calibrateMachineLearningDataset("left-three-emg");
end
