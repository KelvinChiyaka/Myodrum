function detector = initializeKickDetector
% INITIALIZEKICKDETECTOR Create one independent kick detector state.

    detector.mode = 1; % 1 wait downstroke, 2 wait reversal, 3 wait return
    detector.previousAccelMps2 = [];
    detector.recentAccelSteps = zeros(1, 3);
    detector.returnTravelRad = 0;
    detector.lastHitTime = -inf;
    detector.lastSampleTime = NaN;
    detector.previousSignedRate = 0;
    detector.peakDownstrokeRate = 0;
    detector.armedAtTime = -inf;
end
