// Firmware/include/IMU_Control.h
#ifndef IMU_Control
#define IMU_Control

#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

// Initialize IMU (ICM20948)
extern ICM20948_WE imu;

// Kalman filter variables for Roll and Pitch
extern float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[2]; // [estimated angle, uncertainty]

// Time tracking for angle calculation from gyro data
extern unsigned long previousTime;
extern float dt;

// Gyroscope calibration variables
extern float RateCalibrationRoll;
extern float RateCalibrationPitch;
extern float RateCalibrationYaw;
extern int calibrationSamples;

// Offset variables for sensor calibration
extern float accel_x_offset;
extern float accel_y_offset;
extern float accel_z_offset;

// Function declarations
bool initIMU(int maxRetries = 3);
void configIMU();
void calibrateGyroscope();

void calibrateGyroAccel();
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void updateIMUWithKalman();
void setLowNoiseMode();

// Getter functions for filtered angles
float getFilteredRoll();
float getFilteredPitch();
float getFilteredYaw(); // Raw yaw from gyro integration

// Getter functions for raw sensor data
void getRawAccelAngles(float &roll, float &pitch);
void getRawGyroRates(float &x, float &y, float &z);

// New getter functions for sensor data in formats compatible with existing code
void getCalibratedAcceleration(float &x, float &y, float &z);
void getCalibratedGyroscope(float &x, float &y, float &z);
void getMagnetometer(float &x, float &y, float &z);
float getTemperature();

#endif //IMU_Control