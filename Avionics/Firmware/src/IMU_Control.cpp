// Firmware/src/IMU_Control.cpp
#include "IMU_Control.h"

// Initialize IMU object
ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);

// Kalman filter variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4; // 2*2
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4; // 2*2
float Kalman1DOutput[2] = {0, 0}; // [estimated angle, uncertainty]

// Time tracking for angle calculation from gyro data
unsigned long previousTime = 0;
float dt = 0.004;

// Gyroscope calibration variables
float RateCalibrationRoll = 0;
float RateCalibrationPitch = 0;
float RateCalibrationYaw = 0;
int calibrationSamples = 2000; // Number of samples for calibration

// Legacy offset variables for compatibility
float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;
float accel_x_offset = 0;
float accel_y_offset = 0;
float accel_z_offset = 0;


// Cached sensor data for efficiency
static xyzFloat lastGValue;
static xyzFloat lastGyrValue;
static unsigned long lastReadTime = 0;


bool initIMU(int maxRetries) {
  int retryCount = 0;
  while (retryCount < maxRetries) {
      if (imu.init()) {
          Serial.println("ICM20948 found and initialized");
          return true;
      }
      Serial.print("Failed to find ICM20948 chip, retry ");
      Serial.print(retryCount + 1);
      Serial.print(" of ");
      Serial.println(maxRetries);
      retryCount++;
      delay(500);
  }
  Serial.println("CRITICAL: IMU initialization failed after maximum retries");
  return false;
}


void configIMU() {
  // Set the accelerometer range and filter
  imu.setAccRange(ICM20948_ACC_RANGE_2G);
  imu.setAccDLPF(ICM20948_DLPF_6);

  // Set the gyroscope range and filter
  imu.setGyrRange(ICM20948_GYRO_RANGE_250);
  imu.setGyrDLPF(ICM20948_DLPF_6);

  Serial.println("Keep IMU still. Calibrating gyroscope and accelerometer...");

  // Give the sensor some time to stabilize before calibration
  delay(1000);

  // Perform calibrations
  calibrateGyroscope();
  calibrateGyroAccel();

  // Initialize time for dt calculation
  previousTime = micros();

  Serial.println("IMU configuration complete");
}


void calibrateGyroscope() {
  Serial.println("\nBegin Gyroscope Calibration, do not move the sensor...");

  // Reset calibration values
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;

  // Take multiple samples and accumulate
  for (int i = 0; i < calibrationSamples; i++) {
      imu.readSensor();

      xyzFloat gyr = imu.getGyrValues();

      // Accumulate readings
      RateCalibrationRoll += gyr.x;
      RateCalibrationPitch += gyr.y;
      RateCalibrationYaw += gyr.z;

      // Display progress every 200 samples
      if (i % 200 == 0) {
          Serial.print("Calibrating gyroscope... ");
          Serial.print((i * 100) / calibrationSamples);
          Serial.println("%");
      }
      delay(1); // Short delay between readings
  }

  // Calculate average offsets
  RateCalibrationRoll /= calibrationSamples;
  RateCalibrationPitch /= calibrationSamples;
  RateCalibrationYaw /= calibrationSamples;

  Serial.println("Gyroscope Calibration Finished!");
  Serial.print("Gyro Offsets - Roll: ");
  Serial.print(RateCalibrationRoll);
  Serial.print(" Pitch: ");
  Serial.print(RateCalibrationPitch);
  Serial.print(" Yaw: ");
  Serial.println(RateCalibrationYaw);

  // Apply the offsets to the IMU
  imu.setGyrOffsets(RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);
  Serial.println("Gyroscope offsets applied");
}


void calibrateGyroAccel() {
    Serial.println("\nBegin Accelerometer Calibration, do not move the sensor...");
    int num_samples = 400;
    accel_x_offset = 0;
    accel_y_offset = 0;
    accel_z_offset = 0;
    
    for (int i = 0; i < num_samples; i++) {
        imu.readSensor();

        xyzFloat gValue = imu.getGValues();

        // Convert from g to m/s² and accumulate
        accel_x_offset += gValue.x * 9.81;
        accel_y_offset += gValue.y * 9.81;
        accel_z_offset += gValue.z * 9.81;

        if (i % 50 == 0) {
            Serial.print("Calibrating accelerometer... ");
            Serial.print((i * 100) / num_samples);
            Serial.println("%");
        }
        delay(20);
    }
    
    accel_x_offset /= num_samples;
    accel_y_offset /= num_samples;
    accel_z_offset /= num_samples;


    // For Z-axis, subtract gravity (should be ~9.81 m/s² when stationary)
    accel_z_offset -= 9.81;

    Serial.println("Accelerometer Calibration Finished!");
    Serial.print("Accel Offsets - X: ");
    Serial.print(accel_x_offset);
    Serial.print(" Y: ");
    Serial.print(accel_y_offset);
    Serial.print(" Z: ");
    Serial.println(accel_z_offset);
}


void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Prediction step - Project the state aheads
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * 16; // 4*4 process noise variance

  // Update step - Correction based on measurement
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 0); // 3*3 measurement noise variance
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  // Return the results
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}


void updateIMUWithKalman() {
  // Calculate dt for Kalman filter
  unsigned long currentTime = micros();
  dt = (currentTime - previousTime) / 1000000.0; // Convert to seconds
  previousTime = currentTime;

  // Read sensor data
  imu.readSensor();

  // Cache the raw data for other functions
  lastGValue = imu.getGValues();
  lastGyrValue = imu.getGyrValues();
  lastReadTime = currentTime;

  // Calculate angles from accelerometer data
  float AccAngleRoll = atan(lastGValue.y / sqrt(lastGValue.z * lastGValue.z + lastGValue.x * lastGValue.x)) * 180.0 / PI;
  float AccAnglePitch = -atan(lastGValue.z / sqrt(lastGValue.y * lastGValue.y + lastGValue.x * lastGValue.x)) * 180.0 / PI;

  // Apply Kalman filter to Roll angle
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, lastGyrValue.z, AccAngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // Apply Kalman filter to Pitch angle
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, lastGyrValue.y, AccAnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
}


void setLowNoiseMode() {
  // This function exists for compatibility but the new library
  // handles low noise mode through the DLPF settings in configIMU()
  Serial.println("Low noise mode set via DLPF configuration");
}


// Getter functions for filtered angles
float getFilteredRoll() {
  return KalmanAngleRoll;
}


float getFilteredPitch() {
  return KalmanAnglePitch;
}


float getFilteredYaw() {
  // Raw yaw from gyro integration (no magnetometer fusion for now)
  static float integratedYaw = 0;
  integratedYaw += lastGyrValue.z * dt;
  return integratedYaw;
}


// Getter functions for raw sensor data
void getRawAccelAngles(float &roll, float &pitch) {
  roll = atan(lastGValue.y / sqrt(lastGValue.z * lastGValue.z + lastGValue.x * lastGValue.x)) * 180.0 / PI;
  pitch = -atan(lastGValue.z / sqrt(lastGValue.y * lastGValue.y + lastGValue.x * lastGValue.x)) * 180.0 / PI;
}


void getRawGyroRates(float &x, float &y, float &z) {
  x = lastGyrValue.x;
  y = lastGyrValue.y;
  z = lastGyrValue.z;
}


// New compatibility functions for existing FlightData class
void getCalibratedAcceleration(float &x, float &y, float &z) {
  // Convert from g to m/s² and apply calibration offsets
  x = (lastGValue.x * 9.81) - accel_x_offset;
  y = (lastGValue.y * 9.81) - accel_y_offset;
  z = (lastGValue.z * 9.81) - accel_z_offset;
}


void getCalibratedGyroscope(float &x, float &y, float &z) {
  // Gyroscope values are already calibrated by the library
  x = lastGyrValue.x;
  y = lastGyrValue.y;
  z = lastGyrValue.z;
}
