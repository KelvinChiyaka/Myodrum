function players = createKickPlayerBank(audioFile, voiceCount)
% CREATEKICKPLAYERBANK Preload independent voices for low-latency playback.

    arguments
        audioFile (1,1) string
        voiceCount (1,1) double {mustBeInteger, mustBePositive} = 8
    end
    if ~isfile(audioFile)
        error('MyoDrum:MissingKickAudio', 'Kick sample not found: %s', audioFile);
    end
    [audio, sampleRate] = audioread(audioFile);
    peak = max(abs(audio), [], 'all');
    if peak > 1
        audio = audio / peak;
    end
    players = cell(1, voiceCount);
    for voiceIndex = 1:voiceCount
        players{voiceIndex} = audioplayer(audio, sampleRate);
    end
end
