#include <ESP32Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <fstream> 
#include <vector>
#include <sstream>

// -------------------------- Configuration -------------------------- //
#define TRIG_PIN 16
#define ECHO_PIN 17
#define SERVO_PIN 21
#define TARGET_HEIGHT 20.0
#define UPDATE_INTERVAL_MS 50

//IMU Variables
float eulerX, eulerY, eulerZ;
float linearAccelX, linearAccelY, linearAccelZ;
#define I2C_SDA_PIN 				27
#define I2C_SCL_PIN 				25

// PID Variables
float currentHeight = 0.0;
float error = 0.0;
float previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float output = 0.0;

float KP = -0.14;
float KI = -5;
float KD = 0.5;

unsigned long previousTime = 0;
unsigned long oscillationStart = 0;
float oscillationPeriod = 0;
bool isOscillating = false;
int oscillationCount = 0;
float oscillationSum = 0;
bool adaptiveEnabled = true;
float currentSpeed = 0.0; // TODO obtain speed from core

std::string filename = "pid_table.txt"; //file where the pid table is stored
struct PIDTable
{
    float speed;
    float KP;
    float KI;
    float KD;
} pidTable[] = {
    {0.0, -0.14, -5, 0.5},   // Stationary
    {2.0, -0.12, -4.5, 0.4}, // Slow speed
    {4.0, -0.10, -4, 0.3},   // Medium speed
    {6.0, -0.08, -3.5, 0.25} // High speed
    // TODO ask edo Zuanna for the values
};

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_NXPSensorFusion filter;
Adafruit_Sensor_Calibration_EEPROM cal;
Servo servo;

// ---------------------------Function--------------------------- //

void getIMUData(float &pitch, float &roll) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    filter.update(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    pitch = filter.getPitch();
    roll = filter.getRoll();
}

float readHeight() {
    float pitch, roll;
    getIMUData(pitch, roll);
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;

    float pitchRadians = pitch * PI / 180.0;
    float rollRadians = roll * PI / 180.0;

    // Limit correction if angle is too large (sensor reflection issue)
    if (abs(pitch) < 30 && abs(roll) < 30) {
        distance *= cos(pitchRadians) * cos(rollRadians);
    }

    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" Roll: "); Serial.println(roll);
    Serial.print(" Corrected Distance: "); Serial.println(distance);

    return distance;
}
//TODO check if the height is correct

void setServoPosition(float value)
{
    int servoPosition = constrain(value, 0, 180);
    servo.write(servoPosition);
    Serial.println("Servo position set to: " + String(servoPosition));
}

void beginServo()
{
    servo.attach(SERVO_PIN);
    servo.write(90);
}

void beginHC_S04()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void beginIMU() {
	Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
	lsm.begin();
	lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  	lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  	lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}


// What Triggers This Function?
// This function is called when the system detects oscillations.
// The oscillation period (T) is measured in milliseconds (oscillationPeriod).
// If oscillations are detected and measured, the function adjusts the PID gains to improve system stability.
void calculatePID()
{
    if (oscillationPeriod > 0)
    {
        float criticalKP = KP;
        float T = oscillationPeriod / 1000.0;

        KP = 0.6 * criticalKP;
        KI = 1.2 * KP / T;
        KD = 0.075 * KP * T;

        Serial.println("\n--- Ziegler-Nichols Tuning Complete ---");
        Serial.print("KP: ");
        Serial.println(KP);
        Serial.print("KI: ");
        Serial.println(KI);
        Serial.print("KD: ");
        Serial.println(KD);
    }
}

void adaptiveTuning()
{
    if (oscillationCount > 3)
    {
        float avgPeriod = oscillationSum / oscillationCount;
        oscillationSum = 0;
        oscillationCount = 0;

        if (avgPeriod > 0 && adaptiveEnabled)
        {
            float T = avgPeriod / 1000.0;
            KP = 0.6 * KP;
            KI = 1.2 * KP / T;
            KD = 0.075 * KP * T;
            Serial.println("\n--- Adaptive Tuning Applied ---");
            Serial.print("New KP: ");
            Serial.println(KP);
            Serial.print("New KI: ");
            Serial.println(KI);
            Serial.print("New KD: ");
            Serial.println(KD);
        }
    }
}


void updatePIDTable(float speed, float KP, float KI, float KD) {
    std::vector<std::string> lines;
    std::ifstream infile(filename);
    std::ostringstream newLine;
    newLine << speed << " | " << KP << " | " << KI << " | " << KD;
    bool speedExists = false;


    if (infile.is_open()) {
        std::string line;
        while (getline(infile, line)) {
            if (line.find(std::to_string(speed)) == 0) {
                lines.push_back(newLine.str());
                speedExists = true;
            } else {
                lines.push_back(line);
            }
        }
        infile.close();
    }

    if (!speedExists) {
        lines.push_back(newLine.str());
    }

    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << "Speed | KP | KI | KD\n";  
        for (const auto& row : lines) {
            outfile << row << "\n";
        }
        outfile.close();
        Serial.println("PID table updated.");
    } else {
        Serial.println("Error: Unable to open file.");
    }
}

void updatePIDBySpeed(float speed) {
    for (int i = 0; i < sizeof(pidTable) / sizeof(pidTable[0]) - 1; i++) {
        if (speed >= pidTable[i].speed && speed <= pidTable[i + 1].speed) {
            float ratio = (speed - pidTable[i].speed) / (pidTable[i + 1].speed - pidTable[i].speed);
            KP = pidTable[i].KP + ratio * (pidTable[i + 1].KP - pidTable[i].KP);
            KI = pidTable[i].KI + ratio * (pidTable[i + 1].KI - pidTable[i].KI);
            KD = pidTable[i].KD + ratio * (pidTable[i + 1].KD - pidTable[i].KD);

            Serial.println("Interpolated Speed-based PID applied:");
            Serial.print("KP: "); Serial.println(KP);
            Serial.print("KI: "); Serial.println(KI);
            Serial.print("KD: "); Serial.println(KD);

            updatePIDTable(speed, KP, KI, KD); 
            break;
        }
    }
}


void setup()
{
    Serial.begin(115200);
    beginServo();
    beginIMU();
    beginHC_S04();
}

void loop()
{
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= UPDATE_INTERVAL_MS)
    {
        previousTime = currentTime;
        currentHeight = readHeight();
        error = TARGET_HEIGHT - currentHeight;
        integral += error * (UPDATE_INTERVAL_MS / 1000.0);
        derivative = (error - previousError) / (UPDATE_INTERVAL_MS / 1000.0);
        output = (KP * error) + (KI * integral) + (KD * derivative);
        setServoPosition(90 + output);
        updatePIDBySpeed(currentSpeed);

        if (!isOscillating && abs(error) > 2.0)
        {
            isOscillating = true;
            oscillationStart = currentTime;
        }
        else if (isOscillating && abs(error) < 2.0)
        {
            isOscillating = false;
            oscillationPeriod = currentTime - oscillationStart;
            oscillationSum += oscillationPeriod;
            oscillationCount++;
            calculatePID();
            adaptiveTuning();
        }

        previousError = error;

        Serial.print("Speed: ");
        Serial.println(currentSpeed);
        Serial.print("Error: ");
        Serial.println(error);
        Serial.print("P-Term: ");
        Serial.println(KP * error);
        Serial.print("I-Term: ");
        Serial.println(KI * integral);
        Serial.print("D-Term: ");
        Serial.println(KD * derivative);
        Serial.print("Output: ");
        Serial.println(output);
        Serial.print("Oscillation Period (ms): ");
        Serial.println(oscillationPeriod);
    }
}