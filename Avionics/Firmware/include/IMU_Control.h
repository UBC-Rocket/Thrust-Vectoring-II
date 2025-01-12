#ifndef IMU_Control
#define IMU_Control

#include <Adafruit_ICM20948.h>

#define ICM_ADDR 0x68
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG 0x14

//Initialize IMU (ICM20948)
extern Adafruit_ICM20948 imu;

void initIMU();
void configIMU();

// Set low noise modes for both gyroscope and accelerometer
void setLowNoiseMode();

#endif //IMU_Control