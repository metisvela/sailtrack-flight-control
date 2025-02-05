#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <Arduino.h>
#include <ESP32Servo.h>

class MotorController {
public:
    MotorController(uint8_t servoPin, uint16_t minAngle = 0, uint16_t maxAngle = 180);
    bool begin(); // Returns success status
    void setAngle(uint16_t angle);
    uint16_t getCurrentAngle() const { return _currentAngle; }

private:
    Servo _servo;
    const uint8_t _servoPin;
    const uint16_t _minAngle;
    const uint16_t _maxAngle;
    uint16_t _currentAngle = 0;
    bool _attached = false;
};

#endif