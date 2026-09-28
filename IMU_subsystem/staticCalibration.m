function static = staticCalibration(imu1, imu2, c1, c2)
% STATICCALIBRATION Static gyroscope calibration of up to two MPU6050 IMUs.
%
%   static = staticCalibration(imu1, imu2, c1, c2)
%
% Inputs come from setupIMU: the device handles and connected flags.
%
% The IMUs are held still and N samples are averaged. At rest the true
% angular rate is 0, so the mean gyro reading is the gyro bias.
%
% Output struct (row 1 = IMU 1, row 2 = IMU 2; columns = X, Y, Z):
%   static.gyroBias   deg/s   subtract from the gyro reading
%
% The result is also saved to staticCalibration.mat.

    N_SAMPLES    = 500;   % samples averaged
    MAX_GYRO_STD = 0.5;   % deg/s, above this the IMUs probably moved

    %% GYROSCOPE BIAS
    disp('--- GYROSCOPE BIAS ---');
    input('Place both IMUs still on a table, then press Enter...');

    still = averageSamples(imu1, imu2, c1, c2, N_SAMPLES);
    static.gyroBias = still.gyro;

    if any(still.gyroStd(:) > MAX_GYRO_STD)
        warning('IMUs seem to have moved during the gyro test. Repeat it.');
    end

    %% SAVE AND SHOW RESULTS
    save('staticCalibration.mat', 'static');

    disp(' ');
    disp('--- RESULT (row 1 = IMU 1, row 2 = IMU 2; columns X Y Z) ---');
    disp('Gyro bias (deg/s):');
    disp(static.gyroBias);
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