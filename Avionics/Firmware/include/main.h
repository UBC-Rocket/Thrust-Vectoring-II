#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ESP32Servo.h>
#include <PID_v1.h>

#define SERVO_MULTIPLIER 15

// Initialize IMU (ICM20948), servo motors, and servo offsets
extern Servo gimbal_x;
extern Servo gimbal_y;
extern const int servoxinit;
extern const int servoyinit;

extern PID pidX;
extern PID pidY;
extern double setpointX, inputX, outputX;
extern double setpointY, inputY, outputY;

void beginFlight();

#endif //MAIN_H