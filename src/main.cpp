#include <Arduino.h>
#include <TransferFunction.h>
#include <MotorController.h>
#include <SensorManager.h>
#include <cmath>
#include <vector>

// ----------------------- Boat Configuration ------------------------ //
constexpr uint8_t SERVO_PIN = 17;                 // GPIO pin for servo
constexpr uint16_t MAX_ATTACK_ANGLE = 90;         // Mechanical limits
constexpr float WAND_LENGTH_CM = 30.0f;           // Physical wand length
constexpr float MAX_HEIGHT_CM = WAND_LENGTH_CM;   // Theoretical maximum

// Polynomial coefficients (store in PROGMEM for ESP32)
const std::vector<float> CAM_COEFFICIENTS PROGMEM = { 2.009209843887534e-30f, -1.2602855492056582e-27f, 3.3394260263231293e-25f, -4.6286305998480616e-23f, 3.0104053845397855e-21f, 3.9674207040544364e-20f, -2.1879343925607745e-17f, 1.2123459268276988e-15f, 5.673478576600663e-14f, -1.2950284020448881e-11f, 9.855894468088908e-10f, -4.580344113205455e-08f, 1.4466594854605697e-06f, -3.199302856529518e-05f, 0.0004955586436112549f, -0.005277271259996444f, 0.037231829967498235f, -0.16353706319711817f, 0.4024398006534097f, -0.4247071453842014f, 52.41030355496164f };

// ----------------------- System Components ------------------------- //
SensorManager wand;
MotorController flap(SERVO_PIN, 0, MAX_ATTACK_ANGLE);
TransferFunction cam(CAM_COEFFICIENTS);

// ------------------- Flight Control Parameters --------------------- //
constexpr uint32_t CONTROL_LOOP_FREQ = 1;       // 100 Hz update rate
constexpr uint32_t LOOP_INTERVAL_US = 1000000/CONTROL_LOOP_FREQ;
constexpr float ANGLE_DEADBAND = 0.25f;           // 0.25° resolution
constexpr float MAX_ANGLE_RATE = 90.0f;           // 45°/s slew rate limit

// --------------------- Safety Thresholds -------------------------- //
constexpr uint16_t MAX_WAND_ANGLE = 4500;                     // 12-bit sensor max
constexpr float HEIGHT_SANITY_LIMIT = WAND_LENGTH_CM * 1.1f;  // 10% margin

// --------------------- Runtime State ------------------------------ //
volatile uint32_t lastLoopTime = 0;
float lastFlapAngle = 0.0f;

void emergencyHalt() {
    flap.setAngle(0);
    while(1) {
        digitalWrite(LED_BUILTIN, millis()%200 < 100);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    wand.begin();

    if(!flap.begin()) {
        emergencyHalt();
    }

    flap.setAngle(0);
    delay(100);
    if(abs(flap.getCurrentAngle()) > 1.0f) {
        emergencyHalt();
    }

    lastLoopTime = micros();
}

void loop() {
    // Strict timing control using micros()
    uint32_t now = micros();
    if((now - lastLoopTime) < LOOP_INTERVAL_US) return;
    lastLoopTime = now;

    // --- Sensor Acquisition ---
    float wand_angle_deg = wand.movingAverageAngle(1, 0);

    // --- Height Calculation ---
    float height = sin(wand_angle_deg * (M_PI/180.0f)) * WAND_LENGTH_CM;
    
    if(height > HEIGHT_SANITY_LIMIT) {
        emergencyHalt();
    }

    // --- Control Law Application ---
    float target_angle = cam.evaluate(height);
    flap.setAngle(target_angle);

    // --- Flight Safety Envelope ---
    
    // // 1. Slew rate limiting (prevent abrupt movements)
    // float max_delta = MAX_ANGLE_RATE / CONTROL_LOOP_FREQ;
    // target_angle = constrain(target_angle, lastFlapAngle - max_delta, lastFlapAngle + max_delta);

    // // 2. Deadband application
    // if(abs(target_angle - lastFlapAngle) > ANGLE_DEADBAND) {
    //     flap.setAngle(static_cast<uint16_t>(target_angle));
    //     lastFlapAngle = target_angle;
    // }

    Serial.print("Wand angle: ");Serial.print(wand_angle_deg);
    Serial.print(" deg | Boat Height: ");Serial.print(height);
    Serial.print(" cm | Flap angle: ");Serial.print(flap.getCurrentAngle());
    Serial.print(" deg | Target angle: ");Serial.print(target_angle);
    Serial.println(" deg");
}