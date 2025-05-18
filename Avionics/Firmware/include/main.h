// Firmware/include/main.h
#ifndef MAIN_H
#define MAIN_H
#define FIRMWARE_VERSION "1.0.0"

#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"

void beginFlight();

#ifndef MAIN_H_FLIGHT_PHASE
#define MAIN_H_FLIGHT_PHASE

enum FlightPhase {
  IDLE,
  IGNITION,
  POWERED_FLIGHT,
  COASTING,
  APOGEE,
  RECOVERY,
  LANDED
};

extern FlightPhase currentPhase;
#endif // MAIN_H_FLIGHT_PHASE
#endif //MAIN_H