#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_ICM20948.h>
#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

#define ICM_ADDR 0x68
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG 0x14

#define SERVO_MULTIPLIER 15

extern const char *ssid;
extern const char *password;
extern WiFiServer wifiServer;
extern String receivedMessage;
extern bool done;

// Initialize IMU (ICM20948), servo motors, and servo offsets
extern Adafruit_ICM20948 imu;
extern Servo gimbal_x;
extern Servo gimbal_y;
extern const int servoxinit;
extern const int servoyinit;

extern PID pidX;
extern PID pidY;
extern double setpointX, inputX, outputX;
extern double setpointY, inputY, outputY;

void setLowNoiseMode();
void beginFlight();

#endif // MAIN_H