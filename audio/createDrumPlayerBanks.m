function banks = createDrumPlayerBanks(audioFolder, targetNames, ...
        audioFiles, voiceCount)
% CREATEDRUMPLAYERBANKS Preload one polyphonic audio bank per drum target.

    arguments
        audioFolder (1,1) string
        targetNames (1,:) string
        audioFiles (1,:) string
        voiceCount (1,1) double {mustBeInteger, mustBePositive} = 8
    end
    if numel(targetNames) ~= numel(audioFiles)
        error('MyoDrum:InvalidAudioMap', ...
            'Every drum target must have exactly one audio file.');
    end
    banks = struct;
    for targetIndex = 1:numel(targetNames)
        audioFile = fullfile(audioFolder, audioFiles(targetIndex));
        if ~isfile(audioFile)
            error('MyoDrum:MissingDrumAudio', ...
                'Audio sample not found: %s', audioFile);
        end
        [audio, sampleRate] = audioread(audioFile);
        peak = max(abs(audio), [], 'all');
        if peak > 1
            audio = audio / peak;
        end
        voices = cell(1, voiceCount);
        for voiceIndex = 1:voiceCount
            voices{voiceIndex} = audioplayer(audio, sampleRate);
        end
        fieldName = matlab.lang.makeValidName(targetNames(targetIndex));
        banks.(fieldName).players = voices;
        banks.(fieldName).nextVoice = 1;
        banks.(fieldName).audioFile = string(audioFile);
    end
end
