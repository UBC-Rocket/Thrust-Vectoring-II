// include/IMU_Control.h
#ifndef IMU_Control
#define IMU_Control

#include <Adafruit_ICM20948.h>

#define ICM_ADDR 0x68
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG 0x14

//Initialize IMU (ICM20948)
extern Adafruit_ICM20948 imu;

extern float gyro_x_offset;
extern float gyro_y_offset;
extern float gyro_z_offset;
extern float accel_x_offset;
extern float accel_y_offset;
extern float accel_z_offset;

void initIMU();
void configIMU();
void calibrateGyroAccel();

// Set low noise modes for both gyroscope and accelerometer
void setLowNoiseMode();

#endif //IMU_Control