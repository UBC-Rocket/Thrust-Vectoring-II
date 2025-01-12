#include "IMU_Control.h"
#include "flightdata.h"

Adafruit_ICM20948 imu;

void initIMU(){
    if (!imu.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("Failed to find imu20948 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("imu20948 found");
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

void setLowNoiseMode(){
  imu.writeExternalRegister(ICM_ADDR, GYRO_CONFIG_1, 0x01);
  imu.writeExternalRegister(ICM_ADDR, ACCEL_CONFIG, 0x01);
}