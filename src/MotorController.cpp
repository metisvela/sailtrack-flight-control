#include "MotorController.h"

MotorController::MotorController(uint8_t servoPin, uint16_t minAngle, uint16_t maxAngle)
    : _servoPin(servoPin), _minAngle(minAngle), _maxAngle(maxAngle) {}

bool MotorController::begin() {
    if (_servo.attach(_servoPin) == -1) {
        _attached = false;
        return false; // Servo failed to attach
    }
    _attached = true;
    setAngle((_minAngle + _maxAngle) / 2); // Initialize to midpoint
    return true;
}

void MotorController::setAngle(uint16_t angle) {
    if (!_attached) return;
    
    _currentAngle = constrain(angle, _minAngle, _maxAngle);
    _servo.write(static_cast<int>(_currentAngle));
}