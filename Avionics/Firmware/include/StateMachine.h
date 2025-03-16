#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <FS.h>        
#include <SPIFFS.h>

#define BUTTON_PIN 12

// Declare the RocketState enum
enum class RocketState {
    IDLE,     // Waiting for ignition
    IGNITION, // Ignition sequence
    ASCENT,   // Ascent phase
    DESCENT,  // Descent phase
    LANDING   // Landing phase
};

// Declare variables as extern
extern const int PMOS_PIN;
extern const int NMOS_PIN;
extern Adafruit_ICM20948 imu;
extern bool pmosState;
extern bool nmosState;
extern RocketState currentState;

// Declare functions
void setupStateMachine();
void loopStateMachine();
void idleState();
void ignitionState();
void ascentState();
void descentState();
void landingState();
void transitionTo(RocketState newState);
void handleStateChange(RocketState state);
void beginFlight();

#endif // STATEMACHINE_H
