function checksum = crc8EMG(bytes)
% CRC8EMG Calculate CRC-8/ATM (polynomial 0x07, initial value 0).

    arguments
        bytes (:,1) uint8
    end
    checksum = uint8(0);
    polynomial = uint8(7);
    for byteIndex = 1:numel(bytes)
        checksum = bitxor(checksum, bytes(byteIndex));
        for bitIndex = 1:8
            if bitand(checksum, uint8(128)) ~= 0
                checksum = bitxor(bitshift(checksum, 1), polynomial);
            else
                checksum = bitshift(checksum, 1);
            end
        end
    end
end
