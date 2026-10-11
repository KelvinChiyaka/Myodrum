classdef GravityAzimuthEstimator < handle
    % GRAVITYAZIMUTHESTIMATOR Per-sample tilt and azimuth for one stick.
    %
    %   est = GravityAzimuthEstimator(params);
    %   est.update(accelG, gyroRadPerSec, timeSeconds);
    %   features = est.Features;   % [azimuthDeg pitchDeg rollDeg]
    %
    % Tilt (pitch/roll) comes from the gravity direction in the sensor frame.
    % It is propagated with the gyroscope and corrected toward the
    % accelerometer only while the accelerometer reads about 1 g and the
    % stick is not swinging, so strokes and impacts do not pollute it.
    %
    % Azimuth is the angular rate about the vertical (gyro . gravity)
    % integrated with the real sample timestamps. An MPU6050 has no
    % magnetometer, so azimuth is relative and drifts slowly; whenever the
    % stick is still the residual gyro bias is re-estimated and the azimuth
    % is frozen (zero-velocity update). Slow drift that remains is handled by
    % DrumRegionTracker, which corrects it from confident drum hits.
    %
    % Pitch and roll are absolute (gravity referenced) and are NOT affected by
    % recentering; only the azimuth zero moves.

    properties (SetAccess = private)
        GravityBody (1,3) double = [0, 0, 1]
        AzimuthAbsoluteDeg (1,1) double = 0
        NeutralAzimuthDeg (1,1) double = 0
        BiasRadPerSec (1,3) double = [0, 0, 0]
        Initialized (1,1) logical = false
        Converged (1,1) logical = false
        Still (1,1) logical = false
        LastTimeSeconds (1,1) double = NaN
        UpdateCount (1,1) double = 0
        StrokeAxis (1,1) double = 0
    end

    properties (Access = private)
        Params struct
        LastGyroRadPerSec (1,3) double = [0, 0, 0]
        StillSecondsAccumulated (1,1) double = 0
        LastDynamicTime (1,1) double = -inf
        ConvergenceAnchorTiltDeg (1,2) double = [NaN, NaN]
        ConvergenceAnchorTime (1,1) double = NaN
    end

    methods (Static)
        function params = defaultParameters
            params.GravityGainPerSec = 2.0;
            params.AccelGateG = 0.12;
            params.GyroGateRadPerSec = 2.0;
            params.PostDynamicHoldSeconds = 0.25;
            params.MaxStepSeconds = 0.25;
            % Azimuth is only integrated while the stick is not swinging
            % hard or clipping; a stroke is mostly rotation about a
            % horizontal axis and any gravity-direction error would leak
            % into the heading.
            params.AzimuthGyroGateRadPerSec = inf;
            params.ClipRadPerSec = inf;
            params.StillGyroRadPerSec = 0.06;
            params.StillAccelG = 0.06;
            params.StillSeconds = 0.35;
            params.BiasGainPerSec = 0.5;
            params.MaxBiasRadPerSec = deg2rad(1.5);
            params.ConvergenceDeg = 1.5;
            params.ConvergenceSeconds = 1.0;
        end
    end

    methods
        function obj = GravityAzimuthEstimator(params)
            defaults = GravityAzimuthEstimator.defaultParameters;
            if nargin >= 1 && ~isempty(params)
                names = fieldnames(params);
                for nameIndex = 1:numel(names)
                    if isfield(defaults, names{nameIndex})
                        defaults.(names{nameIndex}) = params.(names{nameIndex});
                    end
                end
            end
            obj.Params = defaults;
        end

        function update(obj, accelG, gyroRadPerSec, timeSeconds, freezeAzimuth)
            % freezeAzimuth (optional): hold the heading for this sample.
            % The caller sets it during the impact part of a stroke, where
            % the gyro is large and the heading does not really change.
            if nargin < 5
                freezeAzimuth = false;
            end
            accelG = reshape(double(accelG), 1, 3);
            gyroRadPerSec = reshape(double(gyroRadPerSec), 1, 3);
            if any(~isfinite(accelG)) || any(~isfinite(gyroRadPerSec)) || ...
                    ~isfinite(timeSeconds)
                return;
            end
            accelNorm = norm(accelG);
            p = obj.Params;

            if ~obj.Initialized
                if accelNorm < 0.3
                    return;
                end
                obj.GravityBody = accelG / accelNorm;
                obj.LastGyroRadPerSec = gyroRadPerSec;
                obj.LastTimeSeconds = timeSeconds;
                obj.Initialized = true;
                obj.UpdateCount = 1;
                obj.resetConvergence(timeSeconds);
                return;
            end

            dt = timeSeconds - obj.LastTimeSeconds;
            if dt <= 0
                return;
            end
            dt = min(dt, p.MaxStepSeconds);

            gyroCorrected = gyroRadPerSec - obj.BiasRadPerSec;
            midGyro = 0.5 * (obj.LastGyroRadPerSec + gyroCorrected);
            gyroMagnitude = norm(gyroCorrected);

            % Zero-velocity detection and bias learning.
            isQuietNow = gyroMagnitude < p.StillGyroRadPerSec && ...
                abs(accelNorm - 1) < p.StillAccelG;
            if isQuietNow
                obj.StillSecondsAccumulated = obj.StillSecondsAccumulated + dt;
            else
                obj.StillSecondsAccumulated = 0;
            end
            obj.Still = obj.StillSecondsAccumulated >= p.StillSeconds;
            if obj.Still
                gain = 1 - exp(-p.BiasGainPerSec * dt);
                obj.BiasRadPerSec = obj.BiasRadPerSec + gain * gyroCorrected;
                limit = p.MaxBiasRadPerSec;
                obj.BiasRadPerSec = max(-limit, min(limit, obj.BiasRadPerSec));
            end
            if gyroMagnitude > p.GyroGateRadPerSec
                obj.LastDynamicTime = timeSeconds;
            end

            % Propagate the gravity direction (the sensor frame turns by
            % omega*dt, so a world-fixed vector turns the opposite way).
            previousGravity = obj.GravityBody;
            propagated = GravityAzimuthEstimator.rotate( ...
                previousGravity, -midGyro * dt);
            midGravity = propagated + previousGravity;
            midNorm = norm(midGravity);
            if midNorm > 0
                midGravity = midGravity / midNorm;
            else
                midGravity = previousGravity;
            end

            % Azimuth: rotation rate about the vertical.
            integrateAzimuth = ~obj.Still && ~freezeAzimuth && ...
                gyroMagnitude <= p.AzimuthGyroGateRadPerSec && ...
                max(abs(gyroCorrected)) < p.ClipRadPerSec;
            if integrateAzimuth
                % A stroke is a flexion about one sensor axis. Leaving that
                % axis out removes the stroke from the heading integral
                % (otherwise any small tilt error about that axis leaks the
                % fast, clipped downstroke into azimuth). What remains is
                % rescaled by the share of the vertical it still sees.
                headingGyro = midGyro;
                verticalShare = 1;
                if obj.StrokeAxis >= 1
                    headingGyro(obj.StrokeAxis) = 0;
                    verticalShare = 1 - midGravity(obj.StrokeAxis)^2;
                end
                if verticalShare > 0.2
                    obj.AzimuthAbsoluteDeg = obj.AzimuthAbsoluteDeg + ...
                        rad2deg(dot(headingGyro, midGravity) / ...
                        verticalShare * dt);
                end
            end

            % Gated accelerometer correction.
            correctedGravity = propagated;
            allowCorrection = accelNorm > 0.3 && ...
                (timeSeconds - obj.LastDynamicTime) >= ...
                p.PostDynamicHoldSeconds && gyroMagnitude < p.GyroGateRadPerSec;
            if allowCorrection
                weight = exp(-((accelNorm - 1) / p.AccelGateG)^2);
                blend = 1 - exp(-p.GravityGainPerSec * dt * weight);
                correctedGravity = (1 - blend) * propagated + ...
                    blend * (accelG / accelNorm);
            end
            correctedNorm = norm(correctedGravity);
            if correctedNorm > 0
                obj.GravityBody = correctedGravity / correctedNorm;
            else
                obj.GravityBody = previousGravity;
            end

            obj.LastGyroRadPerSec = gyroCorrected;
            obj.LastTimeSeconds = timeSeconds;
            obj.UpdateCount = obj.UpdateCount + 1;
            obj.trackConvergence(timeSeconds);
        end

        function setStrokeAxis(obj, axisIndex)
            % Sensor axis (1..3) about which strokes flex, or 0 for none.
            if ~ismember(axisIndex, 0:3)
                error('MyoDrum:InvalidStrokeAxis', ...
                    'Stroke axis must be 0 (none), 1, 2 or 3.');
            end
            obj.StrokeAxis = axisIndex;
        end

        function recenter(obj)
            % Make the current heading the zero reference.
            obj.NeutralAzimuthDeg = obj.AzimuthAbsoluteDeg;
        end

        function setAzimuthRelativeDeg(obj, azimuthDeg)
            % Shift the zero reference so the current relative azimuth
            % equals azimuthDeg (used by drift tracking).
            obj.NeutralAzimuthDeg = obj.AzimuthAbsoluteDeg - azimuthDeg;
        end

        function value = AzimuthDeg(obj)
            value = GravityAzimuthEstimator.wrap( ...
                obj.AzimuthAbsoluteDeg - obj.NeutralAzimuthDeg);
        end

        function value = PitchDeg(obj)
            u = obj.GravityBody;
            value = atan2d(-u(1), hypot(u(2), u(3)));
        end

        function value = RollDeg(obj)
            u = obj.GravityBody;
            value = atan2d(u(2), u(3));
        end

        function features = Features(obj)
            features = [obj.AzimuthDeg(), obj.PitchDeg(), obj.RollDeg()];
        end

        function reset(obj)
            obj.GravityBody = [0, 0, 1];
            obj.AzimuthAbsoluteDeg = 0;
            obj.NeutralAzimuthDeg = 0;
            obj.BiasRadPerSec = [0, 0, 0];
            obj.Initialized = false;
            obj.Converged = false;
            obj.Still = false;
            obj.LastTimeSeconds = NaN;
            obj.UpdateCount = 0;
            obj.LastGyroRadPerSec = [0, 0, 0];
            obj.StillSecondsAccumulated = 0;
            obj.LastDynamicTime = -inf;
            obj.ConvergenceAnchorTiltDeg = [NaN, NaN];
            obj.ConvergenceAnchorTime = NaN;
        end
    end

    methods (Access = private)
        function resetConvergence(obj, timeSeconds)
            obj.ConvergenceAnchorTiltDeg = [obj.PitchDeg(), obj.RollDeg()];
            obj.ConvergenceAnchorTime = timeSeconds;
            obj.Converged = false;
        end

        function trackConvergence(obj, timeSeconds)
            tilt = [obj.PitchDeg(), obj.RollDeg()];
            change = tilt - obj.ConvergenceAnchorTiltDeg;
            change(2) = GravityAzimuthEstimator.wrap(change(2));
            if max(abs(change)) > obj.Params.ConvergenceDeg
                obj.ConvergenceAnchorTiltDeg = tilt;
                obj.ConvergenceAnchorTime = timeSeconds;
                obj.Converged = false;
            else
                obj.Converged = (timeSeconds - obj.ConvergenceAnchorTime) >= ...
                    obj.Params.ConvergenceSeconds;
            end
        end
    end

    methods (Static)
        function wrapped = wrap(angleDeg)
            wrapped = mod(angleDeg + 180, 360) - 180;
        end

        function rotated = rotate(vector, rotationVectorRad)
            % Rodrigues rotation of a vector by an axis-angle vector.
            angle = norm(rotationVectorRad);
            if angle < 1e-12
                rotated = vector;
                return;
            end
            axis = rotationVectorRad / angle;
            rotated = vector * cos(angle) + cross(axis, vector) * sin(angle) + ...
                axis * dot(axis, vector) * (1 - cos(angle));
        end
    end
end
