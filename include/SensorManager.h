#ifndef SENSORMANAGER_H_
#define SENSORMANAGER_H_

#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_HIGH 0x0C
#define AS5600_RAW_ANGLE_LOW 0x0D

class SensorManager {
public:
    void begin();
    uint16_t readRawAngle(bool &success); // Return angle + success flag
    uint16_t movingAverageRawAngle(uint8_t n, uint16_t delay_ms);
private:
    uint16_t _lastRawAngle = 0;
};

#endif

/*
NOTES:
1) We can test the use of an exponential moving average (EMA) instead of a simple average.
2) We should consider using non-blocking timers (e.g., millis()) in future iterations
*/