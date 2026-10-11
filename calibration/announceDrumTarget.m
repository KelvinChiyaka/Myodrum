function announceDrumTarget(handName, targetName, hitIndex, hitTotal, phase)
% ANNOUNCEDRUMTARGET Tell the player which drum to hit next.
%
% Kept as its own function so calibration prompts can be observed or
% replaced (for example by tests with simulated sensors).
    arguments
        handName (1,1) string
        targetName (1,1) string
        hitIndex (1,1) double
        hitTotal (1,1) double
        phase (1,1) string = "calibration"
    end
    fprintf('  [%s %d/%d] %s hand -> %s\n', phase, hitIndex, hitTotal, ...
        handName, targetName);
end
