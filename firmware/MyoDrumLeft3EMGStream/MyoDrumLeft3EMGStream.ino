// Synchronized two-MPU6050 + left-arm three-channel EMG streamer for Mega 2560.
// Active EMG inputs: A0, A1 and A2. A4 fills the unused fourth packet slot.
// Commands: S=start, T=stop, P=identify. One 47-byte frame every 5 ms.

#include <Arduino.h>
#include <Wire.h>

constexpr uint32_t SERIAL_BAUD = 250000;
constexpr uint16_t SAMPLE_RATE_HZ = 200;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t FRAME_LENGTH = 47;
constexpr uint8_t IMU_ADDRESSES[] = {0x68, 0x69};
constexpr uint8_t EMG_PINS[] = {A0, A1, A2, A4};

bool streaming = false;
uint16_t sequenceNumber = 0;
uint32_t nextSampleTimeUs = 0;

uint8_t crc8(const uint8_t* bytes, size_t count) {
  uint8_t crc = 0;
  for (size_t index = 0; index < count; ++index) {
    crc ^= bytes[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void writeUint16LE(uint8_t* destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8);
}

void writeUint32LE(uint8_t* destination, uint32_t value) {
  for (uint8_t index = 0; index < 4; ++index) {
    destination[index] = static_cast<uint8_t>(value >> (8 * index));
  }
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readIMUFrame(uint8_t address, uint8_t* destination) {
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t received = Wire.requestFrom(address, static_cast<uint8_t>(14),
                                             static_cast<uint8_t>(true));
  if (received != 14) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t index = 0; index < 14; ++index) {
    destination[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

uint16_t readSettledAnalog(uint8_t pin) {
  analogRead(pin);
  delayMicroseconds(10);
  return analogRead(pin);
}

bool configureIMU(uint8_t address) {
  bool ok = writeRegister(address, 0x6B, 0x00);
  delay(100);
  ok &= writeRegister(address, 0x1A, 0x02);
  ok &= writeRegister(address, 0x19, 0x04);
  ok &= writeRegister(address, 0x1B, 0x10);
  ok &= writeRegister(address, 0x1C, 0x00);
  return ok;
}

void sendFrame(uint32_t timestampUs) {
  uint8_t frame[FRAME_LENGTH] = {0};
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = PROTOCOL_VERSION;
  writeUint16LE(&frame[3], sequenceNumber++);
  writeUint32LE(&frame[5], timestampUs);
  uint8_t status = 0;
  if (!readIMUFrame(IMU_ADDRESSES[0], &frame[9])) status |= 0x01;
  if (!readIMUFrame(IMU_ADDRESSES[1], &frame[23])) status |= 0x02;
  for (uint8_t channel = 0; channel < 4; ++channel) {
    writeUint16LE(&frame[37 + 2 * channel], readSettledAnalog(EMG_PINS[channel]));
  }
  frame[45] = status;
  frame[46] = crc8(&frame[2], 44);
  Serial.write(frame, FRAME_LENGTH);
}

void processCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'S') {
      sequenceNumber = 0;
      nextSampleTimeUs = micros();
      streaming = true;
    } else if (command == 'T') {
      streaming = false;
    } else if (command == 'P') {
      Serial.println(F("MYODRUM_LEFT3_EMG,1,200,1000,2"));
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  Wire.setClock(400000UL);
  analogReference(DEFAULT);
  DIDR0 |= _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC4D);
  for (uint8_t channel = 0; channel < 4; ++channel) {
    pinMode(EMG_PINS[channel], INPUT);
  }
  configureIMU(IMU_ADDRESSES[0]);
  configureIMU(IMU_ADDRESSES[1]);
}

void loop() {
  processCommands();
  if (!streaming) return;
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleTimeUs) >= 0) {
    sendFrame(now);
    nextSampleTimeUs += SAMPLE_PERIOD_US;
    if (static_cast<int32_t>(now - nextSampleTimeUs) >= 0) {
      nextSampleTimeUs = now + SAMPLE_PERIOD_US;
    }
  }
}
