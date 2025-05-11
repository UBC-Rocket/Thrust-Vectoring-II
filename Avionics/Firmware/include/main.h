// Firmware/include/main.h
#ifndef MAIN_H
#define MAIN_H
#define FIRMWARE_VERSION "1.0.0"

#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"

void beginFlight();

#endif //MAIN_H