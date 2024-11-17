#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_ICM20948.h>
#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <ESP32Servo.h>

extern Adafruit_ICM20948 imu;

void setLowNoiseMode();

#endif // MAIN_H