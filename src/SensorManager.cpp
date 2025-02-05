#include "SensorManager.h"

void SensorManager::begin() {
  Wire.begin();
}

uint16_t SensorManager::readRawAngle(bool &success) {
  success = false;

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_HIGH);
  if (Wire.endTransmission(false) != 0) return _lastRawAngle;

  uint8_t bytesReceived = Wire.requestFrom(AS5600_ADDR, 2);
  if (bytesReceived == 2) {
      uint8_t highByte = Wire.read();
      uint8_t lowByte = Wire.read();
      _lastRawAngle = (highByte << 8) | lowByte;
      success = true;
  }
  
  return _lastRawAngle; // Return cached value on failure
}


uint16_t SensorManager::movingAverageRawAngle(uint8_t n, uint16_t delay_ms) {
  uint32_t sum = 0; // Use 32-bit to prevent overflow
  bool success;

  for (uint8_t i = 0; i < n; i++) {
    sum += readRawAngle(success);
    if (!success) i--; // Retry on failure
    delay(delay_ms);
  }

  return (uint16_t)(sum / n);
}