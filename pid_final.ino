#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// === Motor Pins ===
const int stepPin1 = 3;
const int dirPin1 = 6;
const int enPin1 = 8;

const int stepPin2 = 5;
const int dirPin2 = 4;
const int enPin2 = 9;

// === MPU6050 Setup ===
Adafruit_MPU6050 mpu;

// === Filter Parameters ===
float alpha = 0.96;

// === Orientation angles ===
float roll_accel = 0, roll_gyro = 0, roll_filter = 0;

// === Calibration offsets ===
float gyroY_offset = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

// === PID Variables ===
float input;    // Current tilt angle (roll_filter)
float output;   // PID output (motor command)
float setpoint = 0.0; // Upright

float Kp = 390.0;
float Ki = 0.05;
float Kd = 1.00;

float lastError = 0.0;
float integral = 0.0;

// === Timing ===
unsigned long prevTime = 0;

// === Function to calibrate the sensors ===
void calibrateSensors() {
  const int samples = 500;
  float sum_gy = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;

  Serial.println("Calibrating... Keep the sensor still!");


  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum_gy += g.gyro.y;

    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z;

    delay(5);
  }

  gyroY_offset = sum_gy / samples;
  accelX_offset = sum_ax / samples;
  accelY_offset = sum_ay / samples;
  accelZ_offset = (sum_az / samples) - 9.81;

  Serial.println("Calibration complete!");
}

// === Motor Control Function ===
void driveMotors(float motorOutput) {
  static unsigned long lastStepTime = 0;

  float absSpeed = abs(motorOutput);
  unsigned long interval = (absSpeed > 0) ? (1000000.0 / absSpeed) : 1000000;

  if (micros() - lastStepTime >= interval) {
    lastStepTime = micros();

    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
  }

  if (motorOutput >= 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);  // Inverted
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH); // Inverted
  }
}

// === Sensor Reading Function ===
void readSensor() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration
  float corrected_ax = a.acceleration.x - accelX_offset;
  float corrected_ay = a.acceleration.y - accelY_offset;
  float corrected_az = a.acceleration.z - accelZ_offset;
  float corrected_gy = g.gyro.y - gyroY_offset;

  // Calculate roll
  roll_accel = atan2(-corrected_ax, sqrt(corrected_ay * corrected_ay + corrected_az * corrected_az)) * 180 / PI;
  float dt = (millis() - prevTime) / 1000.0;
  prevTime = millis();
  roll_gyro += corrected_gy * dt * 180 / PI;

  roll_filter = alpha * (roll_filter + corrected_gy * dt * 180 / PI) + (1 - alpha) * roll_accel;

  input = roll_filter;
}

// === Handle Serial Input for live PID tuning ===
void handleSerial() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    char param = inputString.charAt(0);
    float value = inputString.substring(1).toFloat();

    if (param == 'p') {
      Kp = value;
      Serial.print("Updated Kp: ");
      Serial.println(Kp);
      Serial.println("=== PID Value Changed! ===");
    } else if (param == 'i') {
      Ki = value;
      Serial.print("Updated Ki: ");
      Serial.println(Ki);
      Serial.println("=== PID Value Changed! ===");
    } else if (param == 'd') {
      Kd = value;
      Serial.print("Updated Kd: ");
      Serial.println(Kd);
      Serial.println("=== PID Value Changed! ===");
    }
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
  calibrateSensors();

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);

  digitalWrite(enPin1, LOW); // Enable motors
  digitalWrite(enPin2, LOW);

  prevTime = millis();
}

// === Main Loop ===
void loop() {
  readSensor();

  // --- PID Calculation ---
  float error = setpoint - input;
  integral += error;
  float derivative = error - lastError;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  output = constrain(output, -500, 500); // Limit motor speed

  driveMotors(output);

  handleSerial(); // Live tuning

  // --- Debugging ---
  Serial.print("Angle: ");
  Serial.print(input);
  Serial.print(" | Output: ");
  Serial.println(output);

  delay(5); // ~200 Hz loop
}
