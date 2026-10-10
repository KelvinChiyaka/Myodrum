function tests = testEMGStream
% TESTEMGSTREAM Hardware-independent firmware-protocol and session tests.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'EMG_subsystem'));
    addpath(fullfile(root, 'drumRunTests', 'emgOnly'));
    addpath(fullfile(root, 'tests', 'helpers'));
    testCase.TestData.Root = root;
end

function testGoodPacketAndBoundaryValues(testCase)
    packet = makePacket(uint16(42), uint32(123456), ...
        uint16([0, 1, 1022, 1023]), uint8(3));
    [frames, remaining, diagnostics, last] = ...
        decodeEMGStreamBytes(packet, []);
    verifyEmpty(testCase, remaining);
    verifyEqual(testCase, frames.sequence, uint16(42));
    verifyEqual(testCase, frames.timestamp_us, uint32(123456));
    verifyEqual(testCase, frames.adc, uint16([0, 1, 1022, 1023]));
    verifyEqual(testCase, frames.status, uint8(3));
    verifyEqual(testCase, last, uint16(42));
    verifyEqual(testCase, diagnostics.checksumFailures, 0);
end

function testGarbageBadChecksumAndResynchronisation(testCase)
    bad = makePacket(uint16(2), uint32(20), uint16(1:4), uint8(0));
    bad(end) = bitxor(bad(end), uint8(1));
    good = makePacket(uint16(3), uint32(30), uint16(5:8), uint8(0));
    bytes = [uint8([9; 8; 7]); bad; good];
    [frames, remaining, diagnostics] = decodeEMGStreamBytes(bytes, []);
    verifyEmpty(testCase, remaining);
    verifyEqual(testCase, frames.sequence, uint16(3));
    verifyEqual(testCase, diagnostics.checksumFailures, 1);
    verifyGreaterThanOrEqual(testCase, diagnostics.parseResets, 1);
end

function testSplitAndBackToBackPackets(testCase)
    first = makePacket(uint16(10), uint32(100), uint16(10:13), uint8(0));
    second = makePacket(uint16(11), uint32(200), uint16(20:23), uint8(0));
    [emptyFrames, remaining] = decodeEMGStreamBytes(first(1:8), []);
    verifyEmpty(testCase, emptyFrames.sequence);
    verifyEqual(testCase, remaining, first(1:8));
    [frames, remaining] = decodeEMGStreamBytes( ...
        [remaining; first(9:end); second], []);
    verifyEmpty(testCase, remaining);
    verifyEqual(testCase, frames.sequence, uint16([10; 11]));
end

function testSequenceWrapAndDroppedFrame(testCase)
    packets = [makePacket(uint16(65535), uint32(1), uint16(1:4), 0); ...
        makePacket(uint16(0), uint32(2), uint16(1:4), 0); ...
        makePacket(uint16(2), uint32(3), uint16(1:4), 0)];
    [frames, ~, diagnostics] = decodeEMGStreamBytes(packets, uint16(65534));
    verifyEqual(testCase, frames.sequence, uint16([65535; 0; 2]));
    verifyEqual(testCase, diagnostics.droppedFrames, 1);
end

function testStreamSessionWritesDeviceFields(testCase)
    folder = string(tempname);
    mkdir(folder);
    cleanup = onCleanup(@()rmdir(folder, 's'));
    cfg = myoDrumConfig(testCase.TestData.Root);
    cfg.Paths.RawData = fullfile(folder, 'raw');
    cfg.Paths.ProcessedData = fullfile(folder, 'processed');
    cfg.Paths.Metadata = fullfile(folder, 'metadata');
    cfg.Paths.Models = fullfile(folder, 'models');
    cfg.EMG.PreflightSeconds = 0.005;
    cfg.Logging.BufferSamples = 50;
    fake = FakeEMGStreamReader;
    mainEMGStreamRecord(Config=cfg, Reader=fake, DurationSeconds=0.02, ...
        SkipRest=true, ShowPlot=false);
    rawFiles = dir(fullfile(cfg.Paths.RawData, 'session_*', ...
        'emgAcquisition.mat'));
    raw = load(fullfile(rawFiles(1).folder, rawFiles(1).name));
    verifyGreaterThan(testCase, size(raw.emgVoltage_V, 1), 0);
    verifyEqual(testCase, size(raw.emgVoltage_V, 2), 4);
    verifyClass(testCase, raw.sequenceNumber, 'uint16');
    verifyClass(testCase, raw.deviceStatus, 'uint8');
    verifyGreaterThan(testCase, diff(raw.time_s), ...
        zeros(numel(raw.time_s) - 1, 1));
    metadataFiles = dir(fullfile(cfg.Paths.Metadata, 'sessions', ...
        'session_*', 'sessionMetadata.mat'));
    saved = load(fullfile(metadataFiles(1).folder, metadataFiles(1).name), ...
        'metadata');
    verifyEqual(testCase, saved.metadata.mode, "emg-stream");
    verifyEqual(testCase, saved.metadata.timestampSource, ...
        "Arduino micros() device clock");
    verifyEqual(testCase, saved.metadata.measuredSetRateHz, 250);
    clear cleanup;
end

function packet = makePacket(sequence, timestamp, adc, status)
    packet = zeros(19, 1, 'uint8');
    packet(1:3) = uint8([170; 85; 1]);
    packet(4:5) = littleEndian(sequence, 2);
    packet(6:9) = littleEndian(timestamp, 4);
    for channel = 1:4
        first = 10 + 2 * (channel - 1);
        packet(first:first + 1) = littleEndian(adc(channel), 2);
    end
    packet(18) = uint8(status);
    packet(19) = crc8EMG(packet(3:18));
end

function bytes = littleEndian(value, count)
    bytes = zeros(count, 1, 'uint8');
    value = uint32(value);
    for index = 1:count
        bytes(index) = uint8(bitand(bitshift(value, -8 * (index - 1)), 255));
    end
end
