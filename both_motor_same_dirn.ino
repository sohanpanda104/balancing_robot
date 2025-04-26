#include <Wire.h>

// === Motor Pins ===
const int stepPin1 = 3;
const int dirPin1 = 6;
const int enPin1 = 8;

const int stepPin2 = 5;
const int dirPin2 = 4;
const int enPin2 = 9;

// === MPU6050 ===
const int MPU = 0x68;

float AccelY, AccelZ, GyroX;
float angle_accel = 0;
float angle_gyro = 0;
float angle_complementary = 0;

unsigned long prevTime;

// === PID Variables ===
float setpoint = 0.0;
float error, previous_error = 0;
float integral = 0;
float derivative;
float output;

// === PID Tuning ===
float Kp = 15.0;
float Ki = 0.0;
float Kd = 1.2;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // MPU6050 Wake up
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Motor Pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);

  digitalWrite(enPin1, LOW); // Enable Motor 1
  digitalWrite(enPin2, LOW); // Enable Motor 2

  prevTime = millis();
}

void stepMotors(float speed) {
  static unsigned long lastStepTime = 0;

  float absSpeed = abs(speed);
  unsigned long interval = (absSpeed > 0) ? (1000000.0 / absSpeed) : 1000000;

  if (micros() - lastStepTime >= interval) {
    lastStepTime = micros();

    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
  }

  // Motor 1: Normal
  // Motor 2: Flipped direction
  if (speed >= 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);  // Inverted
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH); // Inverted
  }
}

void loop() {
  // === Read MPU6050 ===
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // ACCEL_YOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  Wire.read(); Wire.read(); // Skip Ax
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip Temp
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip GyY, GyZ

  // === Convert Units ===
  AccelY = rawAy / 16384.0;
  AccelZ = rawAz / 16384.0;
  GyroX = rawGx / 131.0; // degrees/sec

  // === Accelerometer Angle ===
  angle_accel = atan2(AccelY, AccelZ) * 180 / PI;

  // === Time Delta ===
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // === Complementary Filter ===
  float alpha = 0.98;
  angle_complementary = alpha * (angle_complementary + GyroX * dt) + (1 - alpha) * angle_accel;

  // === PID Controller ===
  error = setpoint - angle_complementary;
  integral += error * dt;
  derivative = (error - previous_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // === Control Motors ===
  stepMotors(output);

  // === Debugging ===
  Serial.print("Angle: ");
  Serial.print(angle_complementary);
  Serial.print(" | PID: ");
  Serial.println(output);

  // Optional: limit loop to ~100 Hz
  delay(10);
}
