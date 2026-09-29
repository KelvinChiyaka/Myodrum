clear;
clc;
close all;
%% SETTINGS
COM_PORT   = 'COM7';
MAX_POINTS = 2000;     % points kept on each plot (older points scroll off)
PLOT_EVERY = 5;        % samples collected between plot/print updates
%% CONNECT TO ARDUINO
disp('MyoDrum IMU System');
disp('Connecting to Arduino Mega 2560...');
a = arduino(COM_PORT, 'Mega2560', 'Libraries', 'I2C');
disp('Arduino connected.');
disp(' ');
%% DETECT AND INITIALISE IMUs
[imu1, imu2, imu1Connected, imu2Connected] = setupIMU(a);
%% STATIC CALIBRATION (gyro bias, runs once, follow the prompt)
% Run last so the gyro bias is measured just before live measurements start.
disp(' ');
still = staticCalibration(imu1, imu2, imu1Connected, imu2Connected);
disp(' ');
disp('Static calibration complete.');
%% SIX-POSE CALIBRATION (accel offset/scale, runs once, follow the prompts)
% sixPositionCalibration.m is a separate function file. It blocks until all
% six poses are done, and stops the script with an error if calibration fails.
disp(' ');
sixPose = sixPoseCalibration(imu1, imu2, imu1Connected, imu2Connected);
disp(' ');
disp('Six-pose calibration complete.');
%% COMBINE CALIBRATIONS
% Accel offset/scale from the six-pose calibration, gyro bias from the
% static calibration. Rows = IMU 1, IMU 2; columns = X, Y, Z.
static.accelOffset = sixPose.accelOffset;
static.accelScale  = sixPose.accelScale;
static.gyroBias    = still.gyroBias;
% The two gyro bias estimates should agree to within a fraction of a deg/s
disp(' ');
disp('Gyro bias difference, static - six-pose (deg/s), should be small:');
disp(still.gyroBias - sixPose.gyroBias);
%% CREATE FIGURE
% Layout: top row = IMU 1 (accel, gyro), bottom row = IMU 2 (accel, gyro)
fig = figure('Name', 'MyoDrum - Dual MPU6050', ...
             'NumberTitle', 'off', ...
             'Color', 'w');
accLines  = cell(1, 2);    % accLines{k}  = X, Y, Z lines for IMU k
gyroLines = cell(1, 2);
for k = 1:2
    accLines{k}  = createPlot(2*k - 1, sprintf('IMU %d - Accelerometer', k), ...
                              'Acceleration (g)', MAX_POINTS);
    gyroLines{k} = createPlot(2*k, sprintf('IMU %d - Gyroscope', k), ...
                              'Angular Velocity (deg/s)', MAX_POINTS);
end
%% BUFFERS (samples wait here until the next plot update)
timeBuf = zeros(PLOT_EVERY, 1);
accBuf  = zeros(PLOT_EVERY, 3, 2);    % (sample, axis, IMU)
gyroBuf = zeros(PLOT_EVERY, 3, 2);
bufCount    = 0;
sampleCount = 0;
%% START MEASUREMENTS
disp(' ');
disp('Starting measurements...');
disp('Close the figure or press Ctrl+C to stop.');
disp(' ');
startTime = tic;
%% MAIN LOOP
while isvalid(fig)
    % --- Read and store one sample (no plotting or printing here) ---
    data = readIMU(imu1, imu2, imu1Connected, imu2Connected);
    imus = {data.IMU1, data.IMU2};
    t    = toc(startTime);
    bufCount    = bufCount + 1;
    sampleCount = sampleCount + 1;
    timeBuf(bufCount) = t;
    for k = 1:2
        if imus{k}.connected
            % Accel: six-pose offset/scale (g). Gyro: static bias removed (deg/s)
            accBuf(bufCount, :, k)  = (imus{k}.accel - static.accelOffset(k, :)) ./ static.accelScale(k, :);
            gyroBuf(bufCount, :, k) = imus{k}.gyro - static.gyroBias(k, :);
        end
    end
    % --- Update plots and print once every PLOT_EVERY samples ---
    if bufCount == PLOT_EVERY
        for k = 1:2
            if imus{k}.connected
                for j = 1:3
                    addpoints(accLines{k}(j),  timeBuf, accBuf(:, j, k));
                    addpoints(gyroLines{k}(j), timeBuf, gyroBuf(:, j, k));
                end
                % Print the newest calibrated sample of this batch
                fprintf(['IMU %d | Acc: X=%6.2f Y=%6.2f Z=%6.2f g | ' ...
                         'Gyro: X=%7.2f Y=%7.2f Z=%7.2f deg/s\n'], ...
                         k, accBuf(end, :, k), gyroBuf(end, :, k));
            end
        end
        drawnow limitrate;
        bufCount = 0;
    end
end
%% FINISHED
disp(' ');
fprintf('Measurement stopped. Average sample rate: %.1f Hz (%d samples in %.1f s)\n', ...
        sampleCount / t, sampleCount, t);
%% ========================= LOCAL FUNCTION =========================
function lines = createPlot(position, plotTitle, yLabelText, maxPoints)
% CREATEPLOT Make one subplot with three animated lines (X, Y, Z).
    subplot(2, 2, position);
    axisNames  = {'X', 'Y', 'Z'};
    axisColors = {'r', 'g', 'b'};
    for j = 1:3
        lines(j) = animatedline('Color', axisColors{j}, ...
                                'LineWidth', 1.2, ...
                                'DisplayName', axisNames{j}, ...
                                'MaximumNumPoints', maxPoints);
    end
    grid on;
    xlabel('Time (s)');
    ylabel(yLabelText);
    title(plotTitle);
    legend('show', 'Location', 'best');
end