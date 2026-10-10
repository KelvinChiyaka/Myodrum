classdef FakeEMGReader < handle
    % FAKEEMGREADER Deterministic test-only analog voltage reader.

    properties
        DelaySeconds (1,1) double = 0
        CallCount (1,1) double = 0
    end

    methods
        function obj = FakeEMGReader(delaySeconds)
            if nargin > 0
                obj.DelaySeconds = delaySeconds;
            end
        end

        function voltage = read(obj, pin)
            if obj.DelaySeconds > 0
                pause(obj.DelaySeconds);
            end
            obj.CallCount = obj.CallCount + 1;
            pinNumber = sscanf(char(pin), 'A%d');
            voltage = 0.1 * pinNumber + 0.01 * mod(obj.CallCount, 5);
        end
    end
end
