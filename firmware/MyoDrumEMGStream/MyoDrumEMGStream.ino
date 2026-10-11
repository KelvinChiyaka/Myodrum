// MyoDrum high-rate four-channel EMG envelope streamer for Arduino Mega 2560.
// Commands: 'S' start, 'T' stop, 'P' identify. No samples are sent until S.

#include <Arduino.h>

constexpr uint32_t SERIAL_BAUD = 250000;
constexpr uint16_t SAMPLE_RATE_HZ = 250;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t CHANNEL_PINS[] = {A0, A2, A3, A4};
constexpr uint8_t CHANNEL_COUNT = sizeof(CHANNEL_PINS) / sizeof(CHANNEL_PINS[0]);
constexpr uint8_t FRAME_LENGTH = 19;

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
  destination[0] = static_cast<uint8_t>(value & 0xFF);
  destination[1] = static_cast<uint8_t>(value >> 8);
}

void writeUint32LE(uint8_t* destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFF);
  destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  destination[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  destination[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint16_t readSettledAnalog(uint8_t pin) {
  // The envelope boards can present more than the AVR ADC's recommended
  // source impedance. After changing channels, the sample/hold capacitor can
  // retain part of the preceding channel. Discard one conversion so the
  // transmitted conversion is acquired after the multiplexer has settled.
  analogRead(pin);
  delayMicroseconds(10);
  return analogRead(pin);
}

void sendFrame(uint32_t timestampUs) {
  uint8_t frame[FRAME_LENGTH];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = PROTOCOL_VERSION;
  writeUint16LE(&frame[3], sequenceNumber++);
  writeUint32LE(&frame[5], timestampUs);
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    writeUint16LE(&frame[9 + 2 * channel],
                  readSettledAnalog(CHANNEL_PINS[channel]));
  }
  frame[17] = 0;  // Status flags reserved for future firmware revisions.
  frame[18] = crc8(&frame[2], 16);
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
      Serial.println(F("MYODRUM_EMG,1,250"));
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  analogReference(DEFAULT);
  // Disable digital input buffers on the four analogue-only signal pins.
  DIDR0 |= _BV(ADC0D) | _BV(ADC2D) | _BV(ADC3D) | _BV(ADC4D);
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    pinMode(CHANNEL_PINS[channel], INPUT);
  }
}

void loop() {
  processCommands();
  if (!streaming) {
    return;
  }
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleTimeUs) >= 0) {
    sendFrame(now);
    nextSampleTimeUs += SAMPLE_PERIOD_US;
    if (static_cast<int32_t>(now - nextSampleTimeUs) >= 0) {
      // Do not emit a burst after a long stall; resume from the current time.
      nextSampleTimeUs = now + SAMPLE_PERIOD_US;
    }
  }
}
