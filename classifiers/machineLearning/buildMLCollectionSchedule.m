function schedule = buildMLCollectionSchedule( ...
        handNames, targetNames, rounds, hitsPerBlock, noHitSeconds)
% BUILDMLCOLLECTIONSCHEDULE Make a balanced, session-aware ML block plan.

    arguments
        handNames (1,:) string
        targetNames (1,:) string
        rounds (1,1) double {mustBeInteger, mustBePositive}
        hitsPerBlock (1,1) double {mustBeInteger, mustBePositive}
        noHitSeconds (1,1) double {mustBePositive}
    end
    if numel(unique(handNames)) ~= numel(handNames) || ...
            numel(unique(targetNames)) ~= numel(targetNames)
        error('MyoDrum:DuplicateMLCollectionLabel', ...
            'Hand and target names must be unique.');
    end
    if any(targetNames == "NO_HIT")
        error('MyoDrum:ReservedMLClassName', ...
            'NO_HIT is reserved for negative examples.');
    end

    classNames = ["NO_HIT", targetNames];
    blockCount = rounds * numel(handNames) * numel(classNames);
    blockId = zeros(blockCount, 1, 'uint16');
    roundIndex = zeros(blockCount, 1, 'uint8');
    handIndex = zeros(blockCount, 1, 'uint8');
    hand = strings(blockCount, 1);
    classIndex = zeros(blockCount, 1, 'uint8');
    label = strings(blockCount, 1);
    expectedHitCount = zeros(blockCount, 1, 'uint16');
    minimumDuration_s = zeros(blockCount, 1);

    row = 0;
    for currentRound = 1:rounds
        if mod(currentRound, 2) == 1
            handOrder = 1:numel(handNames);
        else
            handOrder = numel(handNames):-1:1;
        end
        classOrder = circshift(1:numel(classNames), currentRound - 1);
        for currentHand = handOrder
            for currentClass = classOrder
                row = row + 1;
                blockId(row) = uint16(row);
                roundIndex(row) = uint8(currentRound);
                handIndex(row) = uint8(currentHand);
                hand(row) = handNames(currentHand);
                classIndex(row) = uint8(currentClass);
                label(row) = classNames(currentClass);
                if label(row) == "NO_HIT"
                    minimumDuration_s(row) = noHitSeconds;
                else
                    expectedHitCount(row) = uint16(hitsPerBlock);
                    minimumDuration_s(row) = max(2, 0.25 * hitsPerBlock);
                end
            end
        end
    end

    schedule = table(blockId, roundIndex, handIndex, hand, classIndex, ...
        label, expectedHitCount, minimumDuration_s, ...
        'VariableNames', {'BlockId','Round','HandIndex','Hand', ...
        'ClassIndex','Label','ExpectedHitCount','MinimumDuration_s'});
end
