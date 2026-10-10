# MyoDrum EMG streaming protocol

The Mega 2560 firmware in `firmware/MyoDrumEMGStream` samples the existing
analog envelope outputs at 250 complete four-channel sets per second. It uses
250000 baud, which is exact on the Mega's 16 MHz clock.

## Commands

Commands are single ASCII bytes sent by MATLAB:

- `S`: reset the sequence counter and begin streaming;
- `T`: stop streaming;
- `P`: emit `MYODRUM_EMG,1,250` for manual identification.

The device emits no sample packets until it receives `S`. Do not have a MATLAB
`arduino` object or Arduino Serial Monitor open on the same COM port.

## Binary packet

Each packet is 19 bytes, little-endian:

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 2 | sync `AA 55` |
| 2 | 1 | protocol version (`1`) |
| 3 | 2 | unsigned sequence number |
| 5 | 4 | `micros()` timestamp |
| 9 | 2 | A0 left forearm ADC count |
| 11 | 2 | A2 left triceps ADC count |
| 13 | 2 | A3 right forearm ADC count |
| 15 | 2 | A4 right triceps ADC count |
| 17 | 1 | status flags; currently zero |
| 18 | 1 | CRC-8/ATM over bytes 2 through 17 |

ADC counts are 0-1023 and MATLAB converts them to volts using the configured
ADC reference. The receiver resynchronises after garbage, rejects invalid CRCs,
handles the 16-bit sequence wrap and 32-bit `micros()` wrap, and reports lost
frames rather than silently hiding them.
