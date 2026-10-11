classdef DrumAzimuthTracker < handle
    % DRUMAZIMUTHTRACKER Pick the aimed-at drum and follow slow heading drift.
    %
    % A hit is assigned to the nearest calibrated drum azimuth of that hand
    % after subtracting a per-hand offset. Every confident assignment nudges
    % the offset by a fraction of its residual, so a slow common drift of the
    % MPU6050 heading (no magnetometer) is followed instead of rotating the
    % drum layout away from the player. Uncertain hits are still assigned
    % (nearest drum) but never update the offset.

    properties (SetAccess = private)
        Model struct
        OffsetDeg (1,2) double = [0, 0]
        Lost (1,2) logical = [false, false]
    end

    properties (Access = private)
        Params struct
    end

    methods (Static)
        function params = defaultParameters
            params.UpdateGain = 0.5;
            params.MaxStepDeg = 20;
            params.MaxOffsetDeg = 90;
            params.ConfidentNormalized = 1.6;
            params.MarginNormalized = 1.0;
        end
    end

    methods
        function obj = DrumAzimuthTracker(model, params)
            obj.Model = model;
            defaults = DrumAzimuthTracker.defaultParameters;
            if nargin >= 2 && ~isempty(params)
                names = fieldnames(params);
                for k = 1:numel(names)
                    if isfield(defaults, names{k})
                        defaults.(names{k}) = params.(names{k});
                    end
                end
            end
            obj.Params = defaults;
        end

        function [label, confidence, info] = classify( ...
                obj, handIndex, azimuthDeg, allowUpdate)
            if nargin < 4
                allowUpdate = true;
            end
            h = obj.Model.hands(handIndex);
            corrected = DrumAzimuthTracker.wrap( ...
                azimuthDeg - obj.OffsetDeg(handIndex));
            residuals = DrumAzimuthTracker.wrap(corrected - h.centersDeg);
            normalized = abs(residuals) ./ h.spreadDeg;
            [ordered, order] = sort(normalized);
            best = order(1);
            label = obj.Model.targetNames(best);

            if numel(ordered) > 1
                margin = ordered(2) - ordered(1);
            else
                margin = inf;
            end
            confident = ordered(1) <= obj.Params.ConfidentNormalized && ...
                margin >= obj.Params.MarginNormalized;
            confidence = max(0, min(1, 1 - ordered(1) / ...
                max(obj.Params.ConfidentNormalized * 2, eps))) * ...
                min(1, margin / max(obj.Params.MarginNormalized, eps));

            updated = false;
            if allowUpdate && confident
                step = obj.Params.UpdateGain * residuals(best);
                step = max(-obj.Params.MaxStepDeg, ...
                    min(obj.Params.MaxStepDeg, step));
                newOffset = obj.OffsetDeg(handIndex) + step;
                limit = obj.Params.MaxOffsetDeg;
                obj.Lost(handIndex) = abs(newOffset) > limit;
                obj.OffsetDeg(handIndex) = max(-limit, min(limit, newOffset));
                updated = true;
            end
            info.correctedAzimuthDeg = corrected;
            info.normalizedDistances = normalized;
            info.margin = margin;
            info.confident = confident;
            info.offsetUpdated = updated;
            info.offsetDeg = obj.OffsetDeg(handIndex);
        end

        function reset(obj)
            obj.OffsetDeg = [0, 0];
            obj.Lost = [false, false];
        end
    end

    methods (Static, Access = private)
        function wrapped = wrap(angleDeg)
            wrapped = mod(angleDeg + 180, 360) - 180;
        end
    end
end
