function static = sixPoseCalibration(imu1, imu2, c1, c2)
% SIXPOSITIONCALIBRATION Six-pose static calibration of up to two MPU6050 IMUs.
%
%   static = sixPositionCalibration(imu1, imu2, c1, c2)
%
% Inputs come from setupIMU: the device handles and connected flags.
%
% Each IMU is held still in six poses, with +X, -X, +Y, -Y, +Z and -Z
% pointing up in turn. The same six holds give:
%
%   Accelerometer  An axis pointing up reads +1 g, pointing down -1 g:
%                      reading = scale * (+/-1) + offset
%                      offset  = (up + down) / 2
%                      scale   = (up - down) / 2
%
%   Gyroscope      In every hold the true angular rate is 0, so the mean
%                  gyro reading is the bias. The final bias is the average
%                  over all six holds. Poses come in opposite pairs, so any
%                  bias term that changes with gravity direction cancels.
%
% Output struct (row 1 = IMU 1, row 2 = IMU 2; columns = X, Y, Z):
%   static.gyroBias     deg/s   subtract from the gyro reading
%   static.accelOffset  g       subtract from the accel reading
%   static.accelScale   -       divide the offset-corrected accel by this
%
% The result is also saved to sixPoseCalibration.mat.
% Uses the local function averageSamples (bottom of this file).
    N_SAMPLES    = 500;   % samples averaged per pose
    MAX_GYRO_STD = 0.5;   % deg/s, above this the IMU probably moved
    connected = logical([c1; c2]);
    poseNames = {'+X', '-X', '+Y', '-Y', '+Z', '-Z'};
    poseAccel = cell(1, 6);   % each 2x3 (row = IMU, column = axis)
    poseGyro  = cell(1, 6);
    disp('--- SIX-POSE CALIBRATION ---');
    disp('Rest the IMUs on a table so the named axis points UP.');
    disp('If you use two cases, place them side by side, same orientation.');
    disp(' ');
    %% COLLECT THE SIX POSES
    for p = 1:6
        steady = false;
        while ~steady
            input(sprintf('Pose %d/6: %s axis pointing UP, hold still, press Enter...', ...
                          p, poseNames{p}));
            avg = averageSamples(imu1, imu2, c1, c2, N_SAMPLES);
            % A large gyro spread means the IMU was moving: repeat the pose
            steady = ~any(any(avg.gyroStd(connected, :) > MAX_GYRO_STD));
            if ~steady
                disp('Movement detected. Repeating this pose.');
            end
        end
        poseAccel{p} = avg.accel;
        poseGyro{p}  = avg.gyro;
    end
    %% ACCELEROMETER OFFSET AND SCALE
    static.accelOffset = zeros(2, 3);
    static.accelScale  = ones(2, 3);
    for ax = 1:3
        up   = poseAccel{2*ax - 1}(:, ax);   % 2x1, this axis pointing up
        down = poseAccel{2*ax}(:, ax);       % 2x1, this axis pointing down
        % Sanity check: the axis must have seen about +1 g and -1 g
        if any(up(connected) < 0.5) || any(down(connected) > -0.5)
            error('Axis %d did not read about +/-1 g. Check the pose order.', ax);
        end
        static.accelOffset(connected, ax) = (up(connected) + down(connected)) / 2;
        static.accelScale(connected, ax)  = (up(connected) - down(connected)) / 2;
    end
    %% GYROSCOPE BIAS (average over the six holds)
    allGyro = cat(3, poseGyro{:});                   % 2x3x6
    static.gyroBias = mean(allGyro, 3);
    gyroSpread      = max(allGyro, [], 3) - min(allGyro, [], 3);
    %% SAVE AND SHOW RESULTS
    save('sixPoseCalibration.mat', 'static');
    disp(' ');
    disp('--- RESULTS (row 1 = IMU 1, row 2 = IMU 2; columns X Y Z) ---');
    disp('Accel offset (g):');                  disp(static.accelOffset);
    disp('Accel scale (unitless, ideal = 1):'); disp(static.accelScale);
    disp('Gyro bias (deg/s):');                 disp(static.gyroBias);
    disp('Gyro bias spread between poses (deg/s), should be small:');
    disp(gyroSpread);
end
function avg = averageSamples(imu1, imu2, c1, c2, N)
% AVERAGESAMPLES Average N samples from both IMUs.
%   avg.accel, avg.gyro : 2x3 means (row = IMU, column = axis)
%   avg.gyroStd         : 2x3 gyro standard deviation (deg/s)
    names     = {'IMU1', 'IMU2'};
    connected = [c1, c2];
    accel = zeros(N, 3, 2);
    gyro  = zeros(N, 3, 2);
    for i = 1:N
        data = readIMU(imu1, imu2, c1, c2);
        for k = 1:2
            if connected(k)
                accel(i, :, k) = data.(names{k}).accel;
                gyro(i, :, k)  = data.(names{k}).gyro;
            end
        end
    end
    avg.accel   = squeeze(mean(accel, 1))';
    avg.gyro    = squeeze(mean(gyro, 1))';
    avg.gyroStd = squeeze(std(gyro, 0, 1))';
end