#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Filter parameters
float alpha = 0.96;

// Orientation angles
float pitch_accel, roll_accel;
float pitch_gyro = 0, roll_gyro = 0, yaw_gyro = 0;
float pitch_filter = 0, roll_filter = 0;

// Calibration offsets
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

unsigned long prevTime;

// === Calibration Functions ===
void calibrateSensors() {
  const int samples = 500;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;

  Serial.println("Calibrating... Keep sensor still!");

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum_gx += g.gyro.x;
    sum_gy += g.gyro.y;
    sum_gz += g.gyro.z;

    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z;

    delay(5);
  }

  gyroX_offset = sum_gx / samples;
  gyroY_offset = sum_gy / samples;
  gyroZ_offset = sum_gz / samples;

  accelX_offset = sum_ax / samples;
  accelY_offset = sum_ay / samples;
  accelZ_offset = (sum_az / samples) - 9.81; // Remove gravity (assuming stationary flat)

  Serial.println("Calibration complete!");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000); // Give sensor time to stabilize
  calibrateSensors();

  prevTime = millis();
}

void loop() {
  unsigned long startTime = millis();
  float dt = (startTime - prevTime) / 1000.0; // in seconds
  prevTime = startTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // === Apply Calibration Offsets ===
  float corrected_ax = a.acceleration.x - accelX_offset;
  float corrected_ay = a.acceleration.y - accelY_offset;
  float corrected_az = a.acceleration.z - accelZ_offset;

  float corrected_gx = g.gyro.x - gyroX_offset;
  float corrected_gy = g.gyro.y - gyroY_offset;
  float corrected_gz = g.gyro.z - gyroZ_offset;

  // === Accelerometer angles ===
  pitch_accel = atan2(corrected_ay, corrected_az) * 180 / PI;
  roll_accel = atan2(-corrected_ax, sqrt(corrected_ay * corrected_ay + corrected_az * corrected_az)) * 180 / PI;

  // === Gyroscope angles (integrated) ===
  pitch_gyro += corrected_gx * dt * 180 / PI;
  roll_gyro  += corrected_gy * dt * 180 / PI;
  yaw_gyro   += corrected_gz * dt * 180 / PI;

  // === Complementary filter ===
  pitch_filter = alpha * (pitch_filter + corrected_gx * dt * 180 / PI) + (1 - alpha) * pitch_accel;
  roll_filter  = alpha * (roll_filter + corrected_gy * dt * 180 / PI) + (1 - alpha) * roll_accel;

  // === Print format ===
  // accel_pitch, accel_roll,
  // gyro_pitch, gyro_roll, gyro_yaw,
  // filter_pitch, filter_roll

  Serial.print(pitch_accel); Serial.print(",");
  Serial.print(roll_accel); Serial.print(",");

  Serial.print(pitch_gyro); Serial.print(",");
  Serial.print(roll_gyro); Serial.print(",");
  Serial.print(yaw_gyro); Serial.print(",");

  Serial.print(pitch_filter); Serial.print(",");
  Serial.println(roll_filter);

  // Maintain fixed loop time (e.g., 50ms)
  while (millis() - startTime < 50);
}
