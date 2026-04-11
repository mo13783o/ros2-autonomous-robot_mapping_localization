#include <Wire.h>
#include <MPU9250_WE.h>

#define MPU9250_ADDR 0x68
#define DEG_TO_RAD 0.01745329251

MPU9250_WE imu = MPU9250_WE(MPU9250_ADDR);

// ===== Bias (من الكاليبراشن بتاعك) =====
float gx_bias = 7.368673;
float gy_bias = -5.246443;
float gz_bias = 0.654472;

void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin(8, 9);

  // IMU init
  if (!imu.init()) {
    Serial.println("IMU NOT CONNECTED!");
    while (1);
  }

  imu.setAccRange(MPU9250_ACC_RANGE_2G);
  imu.setGyrRange(MPU9250_GYRO_RANGE_250);

  delay(1000);

  Serial.println("IMU READY");
}

void loop() {

  // ===== Read IMU =====
  xyzFloat g = imu.getGyrValues();        // deg/s
  xyzFloat a = imu.getAccRawValues();     // raw

  // ===== Convert Gyro (rad/s) =====
  float gx = (g.x - gx_bias) * DEG_TO_RAD;
  float gy = (g.y - gy_bias) * DEG_TO_RAD;
  float gz = (g.z - gz_bias) * DEG_TO_RAD;

  // ===== Convert Acc (m/s^2) =====
  float ax = (a.x / 16384.0) * 9.81;
  float ay = (a.y / 16384.0) * 9.81;
  float az = (a.z / 16384.0) * 9.81;

  // ===== Print (ROS-friendly) =====
  Serial.print("IMU ");

  Serial.print(gx, 4); Serial.print(" ");
  Serial.print(gy, 4); Serial.print(" ");
  Serial.print(gz, 4); Serial.print(" ");

  Serial.print(ax, 3); Serial.print(" ");
  Serial.print(ay, 3); Serial.print(" ");
  Serial.println(az, 3);

  delay(10); // ~100 Hz
}