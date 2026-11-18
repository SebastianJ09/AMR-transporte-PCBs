#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <Arduino.h>

// Valores crudos (rango por defecto: +-2 g -> 16384 LSB/g, +-250 dps -> 131 LSB/dps)
struct MPUData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  float temp_c;
};

bool imu_init();
bool imu_read(MPUData &data);
void imu_mpu6050_test_loop();     // prueba unitaria

#endif // IMU_MPU6050_H
