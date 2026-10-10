function mainIMU
% MAINIMU Compatibility entry point for the active two-IMU kick workflow.
%
% The old calibration-and-plot loop moved to
% drumRunTests/diagnostics/mainIMUDiagnostics.m. Running mainIMU now uses the
% cached calibrations and automatically continues to dual kick playback.

    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(projectRoot);
    mainMyoDrum("dual-kick");
end
