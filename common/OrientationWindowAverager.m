classdef OrientationWindowAverager < handle
    % ORIENTATIONWINDOWAVERAGER Average IMU samples over a fixed stride.

    properties (SetAccess = private)
        Stride (1,1) double
        Count (1,1) double = 0
    end

    properties (Access = private)
        AccelSum (1,3) double = [0, 0, 0]
        GyroSum (1,3) double = [0, 0, 0]
    end

    methods
        function obj = OrientationWindowAverager(stride)
            arguments
                stride (1,1) double {mustBeInteger, mustBePositive}
            end
            obj.Stride = stride;
        end

        function [ready, meanAccel, meanGyro] = add(obj, accel, gyro)
            arguments
                obj (1,1) OrientationWindowAverager
                accel (1,3) double {mustBeFinite}
                gyro (1,3) double {mustBeFinite}
            end
            obj.AccelSum = obj.AccelSum + accel;
            obj.GyroSum = obj.GyroSum + gyro;
            obj.Count = obj.Count + 1;
            ready = obj.Count == obj.Stride;
            if ready
                meanAccel = obj.AccelSum / obj.Count;
                meanGyro = obj.GyroSum / obj.Count;
                obj.reset();
            else
                meanAccel = nan(1, 3);
                meanGyro = nan(1, 3);
            end
        end

        function reset(obj)
            obj.Count = 0;
            obj.AccelSum(:) = 0;
            obj.GyroSum(:) = 0;
        end
    end
end
