// Firmware/src/IMU_Control.cpp
#include "IMU_Control.h"

Adafruit_ICM20948 imu;
float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;
float accel_x_offset = 0;
float accel_y_offset = 0;
float accel_z_offset = 0;


bool initIMU(int maxRetries) {
  int retryCount = 0;
  
  while (retryCount < maxRetries) {
    if (imu.begin_I2C(ICM_ADDR, &Wire)) {
      Serial.println("IMU20948 found");
      return true;
    }
    
    Serial.print("Failed to find IMU20948 chip, retry ");
    Serial.print(retryCount + 1);
    Serial.print(" of ");
    Serial.println(maxRetries);
    
    retryCount++;
    delay(500);
  }
  
  Serial.println("CRITICAL: IMU initialization failed after maximum retries");
  return false;
}


void configIMU(){
  setLowNoiseMode();

  /* If rotation in any direction exceeds 250 degrees per second,
  *  then this value will have to be changed. 
  *  However, the lower the value is the more precise the measurement is. */ 
  imu.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);

  /* This rocket will be moving very slowly, 
  *  and acceleration will be far below the minimum range of double Earth's gravity.
  *  This means the highest precision is achieved. */ 
  imu.setAccelRange(ICM20948_ACCEL_RANGE_2_G);

  // Set magnetometer to update 100 times per second
  imu.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  // Set gyro and accel data rate divisors
  imu.setGyroRateDivisor(9); // Gyroscope base output data rate is 9kHz
  imu.setAccelRateDivisor(10);

  Serial.println("Keep IMU still. Calibrating gyroscope and accelerometer...");
  calibrateGyroAccel();
}

void calibrateGyroAccel() {
    int num_samples = 400;
    sensors_event_t gyro_event;
    sensors_event_t accel_event;
    sensors_event_t mag_event;  // placeholder
    sensors_event_t temp_event; // placeholder
    
    for (int i = 0; i < num_samples; i++) {
        imu.getEvent(&accel_event, &gyro_event, &mag_event, &temp_event);
        gyro_x_offset += gyro_event.gyro.x;
        gyro_y_offset += gyro_event.gyro.y;
        gyro_z_offset += gyro_event.gyro.z;
        accel_x_offset += accel_event.acceleration.x;
        accel_y_offset += accel_event.acceleration.y;
        accel_z_offset += accel_event.acceleration.z;
        delay(20);
    }
    gyro_x_offset /= num_samples;
    gyro_y_offset /= num_samples;
    gyro_z_offset /= num_samples;
    accel_x_offset /= num_samples;
    accel_y_offset /= num_samples;
    accel_z_offset /= num_samples;

//     Serial.print("\nGyro offsets - X: "); Serial.print(gyro_x_offset);
//     Serial.print(", Y: "); Serial.print(gyro_y_offset);
//     Serial.print(", Z: "); Serial.println(gyro_z_offset);

//     Serial.print("\nAccel offsets - X: "); Serial.print(accel_x_offset);
//     Serial.print(", Y: "); Serial.print(accel_y_offset);
//     Serial.print(", Z: "); Serial.println(accel_z_offset);
}

void setLowNoiseMode(){
  imu.writeExternalRegister(ICM_ADDR, GYRO_CONFIG_1, 0x01);
  imu.writeExternalRegister(ICM_ADDR, ACCEL_CONFIG, 0x01);
}