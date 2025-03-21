#include <Arduino.h>
#include <TransferFunction.h>
#include <MotorController.h>
#include <SensorManager.h>
#include <cmath>
#include <vector>

// ----------------------- Boat Configuration ------------------------ //
constexpr uint8_t SERVO_PIN = 5;                         // GPIO pin for servo
constexpr uint16_t MAX_ATTACK_ANGLE = 90;                // Mechanical limt
constexpr float VERTICAL_LENGTH_CM = 60.0f;              // Physical vertical length
constexpr float VERTICAL_ARM_LENGTH_CM = 3.0f;           // Physical vertical arm length

// Polynomial coefficients (store in PROGMEM for ESP32)
const std::vector<float> CAM_COEFFICIENTS PROGMEM = { 2.009209843887534e-30f, -1.2602855492056582e-27f, 3.3394260263231293e-25f, -4.6286305998480616e-23f, 3.0104053845397855e-21f, 3.9674207040544364e-20f, -2.1879343925607745e-17f, 1.2123459268276988e-15f, 5.673478576600663e-14f, -1.2950284020448881e-11f, 9.855894468088908e-10f, -4.580344113205455e-08f, 1.4466594854605697e-06f, -3.199302856529518e-05f, 0.0004955586436112549f, -0.005277271259996444f, 0.037231829967498235f, -0.16353706319711817f, 0.4024398006534097f, -0.4247071453842014f, 52.41030355496164f };

// ----------------------- System Components ------------------------- //
SensorManager wand;
MotorController flap(SERVO_PIN, VERTICAL_ARM_LENGTH_CM, 0, MAX_ATTACK_ANGLE);
TransferFunction cam(CAM_COEFFICIENTS);

// ------------------- Flight Control Parameters --------------------- //
constexpr uint32_t CONTROL_LOOP_FREQ = 100;       // 100 Hz update rate
constexpr uint32_t LOOP_INTERVAL_US = 1000000/CONTROL_LOOP_FREQ;

// --------------------- Runtime State ------------------------------ //
volatile uint32_t lastLoopTime = 0;

void emergencyHalt() {
    flap.setAngle(0);
    delay(500);
    ESP.restart();
}

void setup() {
    Serial.begin(115200);

    wand.begin();

    if(!flap.begin()) {
        emergencyHalt();
    }

    flap.setAngle(0);
    delay(500);
    
    lastLoopTime = micros();
}

void loop() {
    // Strict timing control using micros()
    uint32_t now = micros();
    if((now - lastLoopTime) < LOOP_INTERVAL_US) return;
    lastLoopTime = now;

    // --- Sensor Acquisition ---
    float wand_angle_deg = wand.movingAverageAngle(100, 1);

    // --- Control Law Application ---
    float target_length = cam.evaluate(wand_angle_deg);
    flap.setVerticalLength(target_length);

    Serial.printf("Angle: %.2f°, Target: %umm, Servo: %u°\n", wand_angle_deg, target_length, flap.getCurrentAngle());
}