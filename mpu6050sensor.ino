#include <Wire.h>

const int MPU = 0x68;  // MPU6050 I2C address

float AccelX, AccelY, AccelZ;
float GyroX;
float angle_accel = 0;
float angle_gyro = 0;
float angle_complementary = 0;

unsigned long prevTime;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Wake up MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up
  Wire.endTransmission(true);

  prevTime = millis();
}

void loop() {
  // Step 1: Read raw data from MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  int16_t rawAx = Wire.read() << 8 | Wire.read();
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip Temp
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip GyY, GyZ

  // Convert raw values to proper units
  AccelX = rawAx / 16384.0;
  AccelY = rawAy / 16384.0;
  AccelZ = rawAz / 16384.0;
  GyroX  = rawGx / 131.0;  // deg/s

  // Step 2: Calculate angle from accelerometer
  angle_accel = atan2(AccelY, AccelZ) * 180 / PI;

  // Step 3: Calculate angle from gyroscope integration
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  angle_gyro += GyroX * dt;

  // Step 4: Complementary Filter
  float alpha = 0.98;
  angle_complementary = alpha * (angle_complementary + GyroX * dt) + (1 - alpha) * angle_accel;

  // Step 5: Send data over Serial
  Serial.print(angle_accel); Serial.print(",");
  Serial.print(angle_gyro); Serial.print(",");
  Serial.println(angle_complementary);

  // Maintain 100Hz loop (10ms)
  while (millis() - currentTime < 10);
}
