#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <Arduino.h>
#include <ESP32Servo.h>

class MotorController {
public:
    void begin(uint8_t servoPin, uint16_t minAngle = 0, uint16_t maxAngle = 360);
    void setAngle(uint16_t angle);
    uint16_t getCurrentAngle() const;

private:
    Servo _servo;
    uint16_t _currentAngle = 0;
    uint16_t _minAngle = 0;
    uint16_t _maxAngle = 360;
};

#endif