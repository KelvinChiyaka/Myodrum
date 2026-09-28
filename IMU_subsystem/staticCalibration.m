function static = staticCalibration(imu1, imu2, c1, c2)
% STATICCALIBRATION Static calibration of up to two MPU6050 IMUs.
%
%   static = staticCalibration(imu1, imu2, c1, c2)
%
% Inputs come from setupIMUs: the device handles and connected flags.
%
% Part 1: gyroscope bias (IMUs held still)
% Part 2: accelerometer offset and scale (six static poses)
%
% Output struct (row 1 = IMU 1, row 2 = IMU 2; columns = X, Y, Z):
%   static.gyroBias     deg/s   subtract from the gyro reading
%   static.accelOffset  g       subtract from the accel reading
%   static.accelScale   -       divide the offset-corrected accel by this
%
% The result is also saved to imuCalibration.mat.

    N_SAMPLES    = 500;   % samples averaged per pose
    MAX_GYRO_STD = 0.5;   % deg/s, above this the IMUs probably moved

    connected = logical([c1; c2]);

    %% PART 1: GYROSCOPE BIAS
    % At rest the true angular rate is 0, so the mean reading is the bias.
    disp('--- GYROSCOPE BIAS ---');
    input('Place both IMUs still on a table, then press Enter...');

    still = averageSamples(imu1, imu2, c1, c2, N_SAMPLES);
    static.gyroBias = still.gyro;

    if any(still.gyroStd(:) > MAX_GYRO_STD)
        warning('IMUs seem to have moved during the gyro test. Repeat it.');
    end

    %% PART 2: ACCELEROMETER OFFSET AND SCALE
    % An axis pointing up reads +1 g, pointing down reads -1 g.
    %   reading = scale * (+/-1) + offset
    %   offset  = (up + down) / 2
    %   scale   = (up - down) / 2
    disp(' ');
    disp('--- ACCELEROMETER (6 POSES) ---');
    disp('Both IMUs must be fixed to one rigid block with matching axes.');

    poseNames = {'+X up', '-X up', '+Y up', '-Y up', '+Z up', '-Z up'};
    poseMean  = cell(1, 6);

    for p = 1:6
        input(sprintf('Pose %d/6: hold still with %s, then press Enter...', ...
                      p, poseNames{p}));
        avg = averageSamples(imu1, imu2, c1, c2, N_SAMPLES);
        poseMean{p} = avg.accel;     % 2x3 (rows = IMU, columns = axis)
    end

    static.accelOffset = zeros(2, 3);
    static.accelScale  = zeros(2, 3);

    for ax = 1:3
        up   = poseMean{2*ax - 1}(:, ax);   % 2x1, this axis pointing up
        down = poseMean{2*ax}(:, ax);       % 2x1, this axis pointing down

        % Sanity check: the axis must have seen about +1 g and -1 g
        if any(up(connected) < 0.5) || any(down(connected) > -0.5)
            error('Axis %d did not read about +/-1 g. Check the pose order.', ax);
        end

        static.accelOffset(:, ax) = (up + down) / 2;
        static.accelScale(:, ax)  = (up - down) / 2;
    end

    %% SAVE AND SHOW RESULTS
    save('imuCalibration.mat', 'static');

    disp(' ');
    disp('--- RESULTS (row 1 = IMU 1, row 2 = IMU 2; columns X Y Z) ---');
    disp('Gyro bias (deg/s):');                 disp(static.gyroBias);
    disp('Accel offset (g):');                  disp(static.accelOffset);
    disp('Accel scale (unitless, ideal = 1):'); disp(static.accelScale);
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
        data = readIMUI2C(imu1, imu2, c1, c2);
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