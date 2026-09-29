function static = sixPoseCalibration(imu1, imu2, c1, c2)
% SIXPOSECALIBRATION Six-pose static calibration of up to two MPU6050 IMUs.
%
%   static = sixPoseCalibration(imu1, imu2, c1, c2)
%
% Each IMU is held still in six poses, with +X, -X, +Y, -Y, +Z and -Z
% pointing up in turn.
%
% Accelerometer:
%   offset = (up + down) / 2
%   scale  = (up - down) / 2
%
% Gyroscope:
%   The true angular rate is 0 in every pose, so the mean gyro reading
%   over all six poses is used as the gyro bias.
%
% Output:
%   static.gyroBias     deg/s
%   static.accelOffset  g
%   static.accelScale   unitless
%
% The result is also saved to sixPoseCalibration.mat.

    N_SAMPLES    = 500;   % samples averaged per pose
    MAX_GYRO_STD = 0.5;   % deg/s, above this the IMU probably moved

    connected = logical([c1; c2]);

    poseNames = {'+X', '-X', '+Y', '-Y', '+Z', '-Z'};

    poseAccel = cell(1, 6);
    poseGyro  = cell(1, 6);

    disp('--- SIX-POSE CALIBRATION ---');
    disp('Rest the IMUs on a table so the named axis points UP.');
    disp('If you use two cases, place them side by side, same orientation.');
    disp(' ');

    %% COLLECT THE SIX POSES

    for p = 1:6

        steady = false;

        while ~steady

            input(sprintf( ...
                'Pose %d/6: %s axis pointing UP, hold still, press Enter...', ...
                p, poseNames{p}));

            avg = averageSamples( ...
                imu1, imu2, c1, c2, N_SAMPLES);

            % Check whether the IMU moved during the measurement.
            steady = ~any(any( ...
                avg.gyroStd(connected, :) > MAX_GYRO_STD));

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

        % Positive and negative orientations for this axis.
        up   = poseAccel{2*ax - 1}(:, ax);
        down = poseAccel{2*ax}(:, ax);

        % No +/-1 g sanity check is performed here.
        %
        % This means the calibration will continue even if an axis does
        % not measure exactly +1 g and -1 g. The measured values are used
        % directly to calculate the offset and scale.

        static.accelOffset(connected, ax) = ...
            (up(connected) + down(connected)) / 2;

        static.accelScale(connected, ax) = ...
            (up(connected) - down(connected)) / 2;

    end

    %% GYROSCOPE BIAS

    allGyro = cat(3, poseGyro{:});

    % Average gyro measurements from all six stationary poses.
    static.gyroBias = mean(allGyro, 3);

    % Difference between highest and lowest gyro reading for each axis.
    gyroSpread = max(allGyro, [], 3) - min(allGyro, [], 3);

    %% SAVE RESULTS

    save('sixPoseCalibration.mat', 'static');

    %% DISPLAY RESULTS

    disp(' ');
    disp('--- RESULTS (row 1 = IMU 1, row 2 = IMU 2; columns X Y Z) ---');

    disp('Accel offset (g):');
    disp(static.accelOffset);

    disp('Accel scale (unitless, ideal = 1):');
    disp(static.accelScale);

    disp('Gyro bias (deg/s):');
    disp(static.gyroBias);

    disp('Gyro bias spread between poses (deg/s), should be small:');
    disp(gyroSpread);

end


%% ================================================================
% LOCAL FUNCTION: AVERAGE SAMPLES
% ================================================================

function avg = averageSamples(imu1, imu2, c1, c2, N)
% AVERAGESAMPLES Average N samples from both IMUs.
%
%   avg.accel, avg.gyro : 2x3 means
%   avg.gyroStd         : 2x3 gyro standard deviation

    names = {'IMU1', 'IMU2'};

    connected = [c1, c2];

    accel = zeros(N, 3, 2);
    gyro  = zeros(N, 3, 2);

    for i = 1:N

        data = readIMU( ...
            imu1, imu2, c1, c2);

        for k = 1:2

            if connected(k)

                accel(i, :, k) = ...
                    data.(names{k}).accel;

                gyro(i, :, k) = ...
                    data.(names{k}).gyro;

            end

        end

    end

    avg.accel = squeeze(mean(accel, 1))';

    avg.gyro = squeeze(mean(gyro, 1))';

    avg.gyroStd = squeeze( ...
        std(gyro, 0, 1))';

end
