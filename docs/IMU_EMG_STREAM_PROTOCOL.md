# Synchronized IMU + EMG stream

`firmware/MyoDrumIMUEMGStream/MyoDrumIMUEMGStream.ino` owns the Mega serial
port, both MPU6050 devices and all four EMG analogue inputs. It emits one
complete synchronized frame at 200 Hz and 250000 baud. Do not create a MATLAB
`arduino()` object or open Serial Monitor while this mode is running.

Commands are `S` (start), `T` (stop), and `P` (identify). Multi-byte packet
counters and ADC samples are little-endian. MPU register bursts retain their
native big-endian byte order.

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 2 | sync `A5 5A` |
| 2 | 1 | protocol version 1 |
| 3 | 2 | sequence number |
| 5 | 4 | `micros()` timestamp |
| 9 | 14 | MPU6050 0x68 register burst |
| 23 | 14 | MPU6050 0x69 register burst |
| 37 | 8 | A0, A2, A3, A4 ADC counts |
| 45 | 1 | status: bit 0 = 0x68 failure, bit 1 = 0x69 failure |
| 46 | 1 | CRC-8/ATM over offsets 2 through 45 |

The firmware configures both IMUs for 200 Hz, DLPF 2, +/-1000 dps and +/-2 g.
Every EMG channel uses a discarded ADC conversion after multiplexer switching.
MATLAB calibrates rest and active voltage separately for each channel; therefore
different analogue amplifier gains do not share a raw-voltage threshold.
