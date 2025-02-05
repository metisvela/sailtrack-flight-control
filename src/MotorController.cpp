#include "MotorController.h"

void MotorController::begin(uint8_t servoPin, uint16_t minAngle, uint16_t maxAngle) {
    _minAngle = minAngle;
    _maxAngle = maxAngle;
    _servo.attach(servoPin);
}

void MotorController::setAngle(uint16_t angle) {
    _currentAngle = constrain(angle, _minAngle, _maxAngle);
    _servo.write((int)_currentAngle); // Cast to int for library compatibility
}

uint16_t MotorController::getCurrentAngle() const {
    return _currentAngle;
}