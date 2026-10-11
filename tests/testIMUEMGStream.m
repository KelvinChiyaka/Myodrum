function tests = testIMUEMGStream
% TESTIMUEMGSTREAM Hardware-independent combined protocol and fusion tests.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'common'));
    addpath(fullfile(root, 'IMU_subsystem'));
    addpath(fullfile(root, 'EMG_subsystem'));
    testCase.TestData.Root = root;
end

function testGoodFrameAndSignedBoundaries(testCase)
    imu1 = int16([-32768 -1 32767 340 0 328 -328]);
    imu2 = int16([16384 0 -16384 0 32 -32 0]);
    packet = makePacket(42, 123456, imu1, imu2, [0 1 1022 1023], 0);
    [frames, remaining, diagnostics] = ...
        decodeIMUEMGStreamBytes(packet, []);
    verifyEmpty(testCase, remaining);
    verifyEqual(testCase, frames.sequence, uint16(42));
    verifyEqual(testCase, frames.timestamp_us, uint32(123456));
    verifyEqual(testCase, frames.accelG(1,:,1), double(imu1(1:3))/16384, ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, frames.gyroDps(1,:,1), double(imu1(5:7))/32.8, ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, frames.emgAdc, uint16([0 1 1022 1023]));
    verifyEqual(testCase, frames.frameValid, true(1,2));
    verifyEqual(testCase, diagnostics.checksumFailures, 0);
end

function testStatusChecksumSplitAndSequenceGap(testCase)
    values = int16(1:7);
    first = makePacket(65535, 1, values, values, 1:4, 1);
    second = makePacket(1, 2, values, values, 5:8, 2);
    bad = first; bad(end) = bitxor(bad(end), uint8(1));
    [none, remaining] = decodeIMUEMGStreamBytes(first(1:20), []);
    verifyEmpty(testCase, none.sequence);
    bytes = [remaining; first(21:end); bad; second];
    [frames, remaining, diagnostics] = ...
        decodeIMUEMGStreamBytes(bytes, uint16(65534));
    verifyEmpty(testCase, remaining);
    verifyEqual(testCase, frames.sequence, uint16([65535;1]));
    verifyEqual(testCase, frames.frameValid, logical([0 1;1 0]));
    verifyEqual(testCase, diagnostics.checksumFailures, 1);
    verifyEqual(testCase, diagnostics.droppedFrames, 1);
end

function testPerChannelGainNormalization(testCase)
    rest = repmat([0.10 0.50 0.05 1.00], 40, 1);
    left = repmat([0.60 2.50 0.05 1.00], 30, 1);
    right = repmat([0.10 0.50 0.30 4.00], 30, 1);
    active = [left; right];
    hands = ["LEFT";"LEFT";"RIGHT";"RIGHT"];
    profile = buildEMGConfirmationProfile(rest, active, hands, ...
        MinimumActiveSpanVolts=0.05);
    [leftOk, leftScore] = confirmEMGHit(left(1,:), "LEFT", profile, 0.2);
    [rightOk, rightScore] = confirmEMGHit(right(1,:), "RIGHT", profile, 0.2);
    [wrongOk, wrongScore] = confirmEMGHit(right(1,:), "LEFT", profile, 0.2);
    verifyTrue(testCase, leftOk); verifyTrue(testCase, rightOk);
    verifyGreaterThan(testCase, leftScore, 0.9);
    verifyGreaterThan(testCase, rightScore, 0.9);
    verifyFalse(testCase, wrongOk); verifyEqual(testCase, wrongScore, 0);
end

function testProjectExposesCombinedMode(testCase)
    entry = fileread(fullfile(testCase.TestData.Root, 'mainMyoDrum.m'));
    verifyNotEmpty(testCase, regexp(entry, 'imu-emg-stream', 'once'));
    verifyTrue(testCase, isfile(fullfile(testCase.TestData.Root, 'firmware', ...
        'MyoDrumIMUEMGStream', 'MyoDrumIMUEMGStream.ino')));
end

function packet = makePacket(sequence,timestamp,imu1,imu2,adc,status)
    packet=zeros(47,1,'uint8'); packet(1:3)=uint8([165;90;1]);
    packet(4:5)=littleEndian(sequence,2); packet(6:9)=littleEndian(timestamp,4);
    packet(10:23)=bigEndianSigned(imu1); packet(24:37)=bigEndianSigned(imu2);
    for channel=1:4
        at=38+2*(channel-1); packet(at:at+1)=littleEndian(adc(channel),2);
    end
    packet(46)=uint8(status); packet(47)=crc8EMG(packet(3:46));
end
function bytes=bigEndianSigned(values)
    bytes=zeros(2*numel(values),1,'uint8');
    for index=1:numel(values)
        u=typecast(int16(values(index)),'uint16');
        bytes(2*index-1)=uint8(bitshift(u,-8));
        bytes(2*index)=uint8(bitand(u,uint16(255)));
    end
end
function bytes=littleEndian(value,count)
    value=uint32(value); bytes=zeros(count,1,'uint8');
    for index=1:count
        bytes(index)=uint8(bitand(bitshift(value,-8*(index-1)),255));
    end
end
