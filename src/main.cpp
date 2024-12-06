#include <ESP32Servo.h>
#include <SailtrackModule.h>

// -------------------------- Configuration -------------------------- //
// HC-SR04
#define TRIG_PIN                   16
#define ECHO_PIN                   17

// Servo
#define SERVO_PIN                  21

// PID Parameters
#define KP                         -0.14
#define KI                         -5
#define KD                         0.5

#define TARGET_HEIGHT              20.0
#define UPDATE_INTERVAL_MS         50

// Battery Measurement
#define BATTERY_ADC_PIN            35
#define BATTERY_ADC_RESOLUTION     4095
#define BATTERY_ADC_REF_VOLTAGE    1.1
#define BATTERY_ESP32_REF_VOLTAGE  3.3
#define BATTERY_NUM_READINGS       32
#define BATTERY_READING_DELAY_MS   20

// Task Intervals
#define LOOP_TASK_INTERVAL_MS      5
#define MQTT_PUBLISH_FREQ_HZ       5
#define MQTT_TASK_INTERVAL_MS      (1000 / MQTT_PUBLISH_FREQ_HZ)
#define AHRS_UPDATE_FREQ_HZ        5

// PID Variables
float currentHeight = 0.0;
float error = 0.0;
float previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float output = 0.0;

unsigned long previousTime = 0;

// ------------------------------------------------------------------- //
Servo servo;
SailtrackModule stm;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	void onStatusPublish(JsonObject status) {
		JsonObject battery = status.createNestedObject("battery");
		float avg = 0;
		for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
			avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
			delay(BATTERY_READING_DELAY_MS);
		}
		battery["voltage"] = 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
	}
};

void mqttTask(void * pvArguments) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (true) {
		StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
		JsonObject euler = doc.createNestedObject("euler");

		stm.publish("sensor/flight-Control0", doc.as<JsonObjectConst>());
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MQTT_TASK_INTERVAL_MS));
	}
}

float readHeight() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH); 
    float distance = duration * 0.034 / 2;
    return distance;
}

void setServoPosition(float value) {
    int servoPosition = constrain(value, 0, 180);
    servo.write(servoPosition);
    Serial.println("Servo position set to: " + String(servoPosition));
}

void beginServo(){
    servo.attach(SERVO_PIN);
    servo.write(90);
}

void beginHC_S04(){
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void setup() {

    Serial.begin(115200);
    stm.begin("flight-controller", IPAddress(192, 168, 42, 115), new ModuleCallbacks());
    beginServo();
    beginHC_S04();
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= UPDATE_INTERVAL_MS) {
        previousTime = currentTime;
        currentHeight = readHeight();
        Serial.println("Current Height: " + String(currentHeight));
        error = TARGET_HEIGHT - currentHeight;
        integral += error * (UPDATE_INTERVAL_MS / 1000.0);
        derivative = (error - previousError) / (UPDATE_INTERVAL_MS / 1000.0);
        output = (KP * error) + (KI * integral) + (KD * derivative);
        setServoPosition(90 + output);
        previousError = error;
        
        // Debug PID value
        Serial.print("Error: "); Serial.println(error);
        Serial.print("P-Term: "); Serial.println(KP * error);
        Serial.print("I-Term: "); Serial.println(KI * integral);
        Serial.print("D-Term: "); Serial.println(KD * derivative);
        Serial.print("Output: "); Serial.println(output);
    }
    delay(100);
}
