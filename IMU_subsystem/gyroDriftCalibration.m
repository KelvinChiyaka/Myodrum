function drift = gyroDriftCalibration(imu1, imu2, c1, c2, static, opts)
% GYRODRIFTCALIBRATION Characterise gyro drift and noise of up to two MPU6050 IMUs.
%
%   drift = gyroDriftCalibration(imu1, imu2, c1, c2, static)
%   drift = gyroDriftCalibration(imu1, imu2, c1, c2, static, opts)
%
% The IMUs are kept stationary while gyro data are logged at a fixed,
% scheduled sample interval. The static bias is removed and the residual is
% analysed with:
%   - descriptive statistics (mean, detrended std, min, max)
%   - a linear fit to 1 s block means (drift rate with 95% CI)
%   - overlapping Allan deviation (ARW, bias instability)
%   - regression against die temperature (if readIMU provides it)
%
% Inputs:
%   imu1, imu2 : MPU6050 device handles
%   c1, c2     : connection flags
%   static     : struct with static.gyroBias (2x3, deg/s)
%   opts       : optional struct
%                opts.duration (s, analysed time after warm-up, default 1800)
%                opts.warmup   (s, recorded but excluded from analysis, default 300)
%                opts.fs       (Hz, nominal sample rate, default 100)
%                opts.saveFile (default 'gyroDriftCalibration.mat')
%
% Assumptions about readIMU (not verified here):
%   data.IMU1.gyro / data.IMU2.gyro are 1x3 in deg/s.
%   data.IMUk.temp (deg C) is used if present; otherwise temperature is NaN.
%
% Output fields (IMU index first, axis second, unless stated):
%   drift.time              Sample timestamps (s), Nx1 (includes warm-up)
%   drift.rawGyro           Raw gyro, Nx3x2 (deg/s), NaN for unconnected IMU
%   drift.correctedGyro     Raw minus static bias, Nx3x2 (deg/s)
%   drift.temp              Die temperature, Nx2 (deg C)
%   drift.timing            Sampling diagnostics (struct)
%   drift.mean              Mean residual after warm-up (deg/s)
%   drift.std               Std of residual (deg/s), includes any trend
%   drift.stdDetrended      Std after linear detrend (deg/s), noise estimate
%   drift.min, drift.max    Extremes of residual (deg/s)
%   drift.slope             Linear drift rate (deg/s per min)
%   drift.slopeCI95         Approx. 95% half-width of slope (deg/s per min)
%   drift.rho1              Lag-1 autocorrelation of block-mean fit residuals
%   drift.tempCoeff         d(residual)/d(temperature) (deg/s per deg C)
%   drift.allan             tau, adev, err (nTau x 3 x 2), ARW, BI, tauBI, biResolved
%
% Results are saved to opts.saveFile.

    %% OPTIONS
    if nargin < 6 || isempty(opts), opts = struct(); end
    DURATION  = getOpt(opts, 'duration', 1800);
    WARMUP    = getOpt(opts, 'warmup',   300);
    FS_NOM    = getOpt(opts, 'fs',       100);
    SAVE_FILE = getOpt(opts, 'saveFile', 'gyroDriftCalibration.mat');
    BLOCK_S   = 1;                 % block-mean length for the linear fit (s)
    MIN_ANALYSIS_S = 60;           % minimum post-warm-up record accepted

    connected = logical([c1, c2]);

    %% CHECK INPUT
    if ~isfield(static, 'gyroBias')
        error('The static calibration structure must contain static.gyroBias.');
    end
    if ~isequal(size(static.gyroBias), [2 3])
        error('static.gyroBias must be a 2x3 matrix.');
    end

    %% START TEST
    Ts    = 1 / FS_NOM;
    Ttot  = WARMUP + DURATION;
    Nmax  = floor(Ttot * FS_NOM) + 1;

    disp(' ');
    disp('==========================================');
    disp('        GYRO DRIFT CALIBRATION');
    disp('==========================================');
    fprintf('Warm-up (excluded from analysis): %.1f min\n', WARMUP / 60);
    fprintf('Analysed duration:                %.1f min\n', DURATION / 60);
    fprintf('Nominal sample rate:              %.1f Hz\n', FS_NOM);
    disp(' ');
    disp('Place both IMUs on a stable surface. Do NOT touch the IMUs,');
    disp('the cables or the table during the test. Keep airflow and');
    disp('temperature changes in the room to a minimum.');
    disp(' ');
    input('Place both IMUs still, then press Enter to start...');

    %% PRE-ALLOCATE (exact size; no growth inside the loop)
    tStamp = nan(Nmax, 1);      % midpoint of read interval (s)
    tRead  = nan(Nmax, 1);      % duration of each readIMU call (s)
    raw    = nan(Nmax, 3, 2);
    temp   = nan(Nmax, 2);

    %% ACQUIRE (absolute-time schedule, no plotting)
    n = 0;
    nextReport = 30;
    t0 = tic;

    for i = 1:Nmax

        tSched = (i - 1) * Ts;
        while toc(t0) < tSched      % spin until the sample is due
        end

        tPre = toc(t0);
        try
            data = readIMU(imu1, imu2, c1, c2);
        catch ME
            warning('readIMU failed at t = %.1f s (%s). Analysing partial record.', ...
                tPre, ME.message);
            break;
        end
        tPost = toc(t0);

        n = i;
        tStamp(i) = 0.5 * (tPre + tPost);
        tRead(i)  = tPost - tPre;

        if c1
            raw(i, :, 1) = data.IMU1.gyro;
            temp(i, 1)   = getTemp(data.IMU1);
        end
        if c2
            raw(i, :, 2) = data.IMU2.gyro;
            temp(i, 2)   = getTemp(data.IMU2);
        end

        if tPost >= nextReport
            fprintf('  t = %6.1f s / %.0f s\n', tPost, Ttot);
            nextReport = nextReport + 30;
        end
    end

    if n < 100
        error('Fewer than 100 samples acquired; cannot analyse.');
    end

    %% TRIM
    t      = tStamp(1:n);
    tRead  = tRead(1:n);
    raw    = raw(1:n, :, :);
    temp   = temp(1:n, :);

    %% REMOVE STATIC BIAS (vectorised; b is 1x3x2)
    b = permute(static.gyroBias, [3 2 1]);
    corrected = raw - b;

    %% TIMING DIAGNOSTICS
    dt       = diff(t);
    achieved = (n - 1) / (t(end) - t(1));

    timing.achievedRate    = achieved;
    timing.dtMean          = mean(dt);
    timing.dtStd           = std(dt);
    timing.dtMax           = max(dt);
    timing.lateFraction    = mean(dt > 1.5 * Ts);
    timing.readTimeMean    = mean(tRead);
    timing.readTimeMax     = max(tRead);
    timing.duplicateFraction = nan(1, 2);
    for k = find(connected)
        timing.duplicateFraction(k) = ...
            mean(all(diff(raw(:, :, k), 1, 1) == 0, 2));
    end

    if achieved < 0.98 * FS_NOM
        warning(['Achieved rate %.1f Hz is below nominal %.1f Hz. ' ...
                 'Data will be resampled to a uniform grid at the achieved rate.'], ...
                 achieved, FS_NOM);
    end

    %% UNIFORM GRID FOR ANALYSIS (post warm-up only)
    Fs     = min(FS_NOM, round(achieved));
    tStart = max(WARMUP, t(1));
    if t(end) - tStart < MIN_ANALYSIS_S
        error('Post-warm-up record is shorter than %d s.', MIN_ANALYSIS_S);
    end
    tU   = (tStart : 1/Fs : t(end))';
    blen = round(Fs * BLOCK_S);              % samples per block
    nb   = floor(numel(tU) / blen);          % number of complete blocks
    tb   = tU(1) + ((0:nb-1)' + 0.5) * BLOCK_S;
    tc   = tb - mean(tb);                    % centred time for regression

    %% PRE-ALLOCATE RESULTS
    drift.mean = nan(2, 3);  drift.std = nan(2, 3);
    drift.stdDetrended = nan(2, 3);
    drift.min  = nan(2, 3);  drift.max = nan(2, 3);
    drift.slope = nan(2, 3); drift.slopeCI95 = nan(2, 3);
    drift.rho1  = nan(2, 3); drift.tempCoeff = nan(2, 3);

    blockMean = nan(nb, 3, 2);
    tempBlock = nan(nb, 2);
    adev = []; aerr = []; tau = [];
    ARW = nan(2, 3); BI = nan(2, 3); tauBI = nan(2, 3);
    biResolved = false(2, 3);

    %% ANALYSE
    for k = find(connected)

        % Temperature on the same grid
        hasTemp = any(~isnan(temp(:, k)));
        if hasTemp
            tempU = interp1(t, temp(:, k), tU, 'linear');
            tempBlock(:, k) = mean(reshape(tempU(1:nb*blen), blen, nb), 1)';
        end
        tempRangeOK = hasTemp && (max(tempBlock(:, k)) - min(tempBlock(:, k)) > 0.5);

        for a = 1:3

            yU = interp1(t, corrected(:, a, k), tU, 'linear');

            % --- Descriptive statistics
            drift.mean(k, a)         = mean(yU);
            drift.std(k, a)          = std(yU);
            drift.stdDetrended(k, a) = std(detrend(yU, 1));
            drift.min(k, a)          = min(yU);
            drift.max(k, a)          = max(yU);

            % --- Linear fit to block means, with AR(1)-inflated SE
            bm = mean(reshape(yU(1:nb*blen), blen, nb), 1)';
            blockMean(:, a, k) = bm;

            X    = [ones(nb, 1), tc];
            beta = X \ bm;
            res  = bm - X * beta;
            s2   = sum(res.^2) / (nb - 2);
            se   = sqrt(s2 / sum(tc.^2));
            rho  = sum(res(1:end-1) .* res(2:end)) / sum(res.^2);
            rho  = min(max(rho, 0), 0.99);
            ci   = 1.96 * se * sqrt((1 + rho) / (1 - rho));

            drift.slope(k, a)     = beta(2) * 60;   % deg/s per s -> deg/s per min
            drift.slopeCI95(k, a) = ci * 60;
            drift.rho1(k, a)      = rho;

            % --- Temperature coefficient (indicative: collinear with time)
            if tempRangeOK
                p = polyfit(tempBlock(:, k), bm, 1);
                drift.tempCoeff(k, a) = p(1);
            end

            % --- Allan deviation
            [tau, s, e] = allanDev(yU, 1 / Fs);
            if isempty(adev)
                adev = nan(numel(tau), 3, 2);
                aerr = nan(numel(tau), 3, 2);
            end
            adev(:, a, k) = s;
            aerr(:, a, k) = e;

            % ARW = sigma(tau = 1 s) in deg/sqrt(s); log-log interpolation
            if tau(1) <= 1 && tau(end) >= 1
                ARW(k, a) = exp(interp1(log(tau), log(s), 0));
            end

            % Bias instability = min(sigma) / 0.664 (flicker noise, IEEE 952)
            [sMin, iMin] = min(s);
            BI(k, a)     = sMin / 0.664;
            tauBI(k, a)  = tau(iMin);
            biResolved(k, a) = iMin > 1 && iMin < numel(tau) - 1;
        end
    end

    %% STORE
    drift.time          = t;
    drift.rawGyro       = raw;
    drift.correctedGyro = corrected;
    drift.temp          = temp;
    drift.timing        = timing;
    drift.analysis.Fs        = Fs;
    drift.analysis.tStart    = tStart;
    drift.analysis.warmup    = WARMUP;
    drift.allan.tau          = tau;
    drift.allan.adev         = adev;
    drift.allan.err          = aerr;
    drift.allan.ARW          = ARW;     % deg/sqrt(s)
    drift.allan.BI           = BI;      % deg/s
    drift.allan.tauBI        = tauBI;   % s
    drift.allan.biResolved   = biResolved;

    %% DISPLAY
    disp(' ');
    disp('==========================================');
    disp('          GYRO DRIFT RESULTS');
    disp('==========================================');
    fprintf('\nTiming: achieved %.2f Hz | dt mean %.2f ms, std %.2f ms, max %.1f ms\n', ...
        timing.achievedRate, 1e3*timing.dtMean, 1e3*timing.dtStd, 1e3*timing.dtMax);
    fprintf('        late samples %.2f %% | read time mean %.2f ms, max %.1f ms\n', ...
        100*timing.lateFraction, 1e3*timing.readTimeMean, 1e3*timing.readTimeMax);
    fprintf('        analysis rate %d Hz over %.1f s (after %.0f s warm-up)\n', ...
        Fs, t(end) - tStart, WARMUP);

    axNames = 'XYZ';
    for k = find(connected)
        fprintf('\nIMU %d  (duplicate samples: %.2f %%)\n', k, 100*timing.duplicateFraction(k));
        if any(~isnan(temp(:, k)))
            fprintf('  Die temperature: start %.2f, end %.2f, range %.2f deg C\n', ...
                temp(1, k), temp(end, k), max(temp(:, k)) - min(temp(:, k)));
        else
            fprintf('  Die temperature: not available from readIMU\n');
        end
        fprintf('  Axis | mean(dps) | noise std(dps) | slope(dps/min) +/- 95%%CI | ARW(deg/rt-hr) | BI(deg/hr) @tau(s)\n');
        for a = 1:3
            flag = '';
            if ~biResolved(k, a), flag = ' (min not resolved)'; end
            fprintf('   %c   | %9.4f | %14.4f | %+10.5f +/- %.5f   | %14.3f | %10.2f @ %.0f%s\n', ...
                axNames(a), drift.mean(k,a), drift.stdDetrended(k,a), ...
                drift.slope(k,a), drift.slopeCI95(k,a), ...
                60*ARW(k,a), 3600*BI(k,a), tauBI(k,a), flag);
        end
    end

    %% FIGURES (after acquisition, so plotting cannot disturb timing)
    nC = nnz(connected);
    f1 = figure('Name', 'MyoDrum - Gyro residual', 'NumberTitle', 'off', 'Color', 'w');
    f2 = figure('Name', 'MyoDrum - Allan deviation', 'NumberTitle', 'off', 'Color', 'w');
    f3 = figure('Name', 'MyoDrum - Sampling timing', 'NumberTitle', 'off', 'Color', 'w');

    r = 0;
    for k = find(connected)
        r = r + 1;

        figure(f1);
        subplot(nC, 1, r);
        yyaxis left;
        h = plot(tb, blockMean(:, :, k));
        ylabel('Residual, 1 s mean (deg/s)');
        if any(~isnan(tempBlock(:, k)))
            yyaxis right;
            plot(tb, tempBlock(:, k), 'k--');
            ylabel('Die temperature (deg C)');
        end
        grid on; xlabel('Time (s)');
        title(sprintf('IMU %d - residual after static bias removal (post warm-up)', k));
        legend(h, {'X', 'Y', 'Z'}, 'Location', 'best');

        figure(f2);
        subplot(nC, 1, r);
        hold on;
        for a = 1:3
            errorbar(tau, adev(:, a, k), aerr(:, a, k), '.-');
        end
        set(gca, 'XScale', 'log', 'YScale', 'log');
        grid on;
        xlabel('\tau (s)'); ylabel('\sigma(\tau) (deg/s)');
        title(sprintf('IMU %d - overlapping Allan deviation', k));
        legend({'X', 'Y', 'Z'}, 'Location', 'best');
    end

    figure(f3);
    subplot(2, 1, 1);
    histogram(1e3 * dt);
    set(gca, 'YScale', 'log'); grid on;
    xlabel('Sample interval (ms)'); ylabel('Count');
    title(sprintf('Sample interval (nominal %.1f ms)', 1e3 * Ts));
    subplot(2, 1, 2);
    plot(t(1:end-1), 1e3 * tRead(1:end-1)); grid on;
    xlabel('Time (s)'); ylabel('readIMU duration (ms)');
    title('Read time per sample');

    %% SAVE
    save(SAVE_FILE, 'drift');
    disp(' ');
    fprintf('Gyro drift calibration complete. Results saved to %s\n', SAVE_FILE);

end

%% ------------------------------------------------------------------------
function [tau, adev, err] = allanDev(y, tau0)
% ALLANDEV Overlapping Allan deviation of a rate signal y sampled every tau0.
%   The rate is integrated to angle x (N+1 points, x(1) = 0) and the second
%   difference of x at lag m is used:
%     sigma^2(m*tau0) = sum_k (x_{k+2m} - 2 x_{k+m} + x_k)^2
%                       / (2 (m*tau0)^2 (N+1-2m))
%   Error bar: sigma / sqrt(2 (N/m - 1)) (IEEE Std 952-1997, non-overlapping
%   expression; conservative for the overlapping estimator).

    N    = numel(y);
    x    = [0; cumsum(y(:)) * tau0];          % deg
    mMax = floor(N / 9);                      % >= ~9 clusters at the largest tau
    m    = unique(round(logspace(0, log10(mMax), 40)))';
    tau  = m * tau0;
    adev = nan(size(m));

    for j = 1:numel(m)
        mj = m(j);
        d  = x(1+2*mj:end) - 2 * x(1+mj:end-mj) + x(1:end-2*mj);
        adev(j) = sqrt(sum(d.^2) / (2 * tau(j)^2 * numel(d)));
    end

    err = adev ./ sqrt(2 * (N ./ m - 1));
end

function v = getOpt(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end

function T = getTemp(s)
    try
        T = s.temp;
    catch
        try
            T = s.temperature;
        catch
            T = NaN;
        end
    end
end