#include "MotorController.h"

MotorController::MotorController(uint8_t servoPin, uint16_t armLength, uint16_t minAngle, uint16_t maxAngle)
    : _servoPin(servoPin), _minAngle(minAngle), _maxAngle(maxAngle), _armLength(armLength) {}

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

void MotorController::setVerticalLength(uint16_t length) {
    if (!_attached) return;

    float angleRadians = asin(static_cast<float>(length) / _armLength);
    uint16_t angleDegrees = static_cast<uint16_t>(angleRadians * 180.0 / M_PI);

    setAngle(angleDegrees);
}