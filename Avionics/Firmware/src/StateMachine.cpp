#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <FS.h>
#include <SPIFFS.h>
#include "StateMachine.h"

// Define the pins for ignition control
#define BUTTON_PIN 12
const int PMOS_PIN = 26;  // Force external linkage
const int NMOS_PIN = 25;  // Force external linkage

// Define the RocketState enum only once in the header (do not redefine here)
// Current rocket state
RocketState currentState = RocketState::IDLE;

// Global variables for ignition states
extern bool pmosState;
extern bool nmosState;

void setupStateMachine() {
    Serial.begin(115200);
    while (!Serial) {}

    // Initialize the IMU
    if (!imu.begin_I2C()) {
        Serial.println("Failed to find IMU");
        while (true);
    }
    Serial.println("IMU found");

    // Initialize Pins
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PMOS_PIN, OUTPUT);
    pinMode(NMOS_PIN, OUTPUT);

    currentState = RocketState::IDLE;
}

void loopStateMachine() {
    switch (currentState) {
        case RocketState::IDLE:
            idleState();
            break;
        case RocketState::IGNITION:
            ignitionState();
            break;
        case RocketState::ASCENT:
            ascentState();
            break;
        case RocketState::DESCENT:
            descentState();
            break;
        case RocketState::LANDING:
            landingState();
            break;
    }
}

void idleState() {
    Serial.println("In IDLE state.");
    if (digitalRead(BUTTON_PIN) == HIGH) {
        transitionTo(RocketState::IGNITION);
    }
}

void ignitionState() {
    Serial.println("In IGNITION state.");
    digitalWrite(PMOS_PIN, !pmosState);
    digitalWrite(NMOS_PIN, !nmosState);
    pmosState = !pmosState;
    nmosState = !nmosState;
    delay(2000);
    transitionTo(RocketState::ASCENT);
}

void ascentState() {
    Serial.println("In ASCENT state.");
    sensors_event_t accel, gyro;
    imu.getEvent(&accel, &gyro, nullptr, nullptr);
    Serial.print("Acceleration: ");
    Serial.print(accel.acceleration.x);
    Serial.print(", ");
    Serial.print(accel.acceleration.y);
    Serial.print(", ");
    Serial.println(accel.acceleration.z);
    transitionTo(RocketState::DESCENT);
}

void descentState() {
    Serial.println("In DESCENT state.");
    delay(5000);
    transitionTo(RocketState::LANDING);
}

void landingState() {
    Serial.println("In LANDING state.");
    transitionTo(RocketState::IDLE);
}

void transitionTo(RocketState newState) {
    currentState = newState;
    handleStateChange(newState);
}

void handleStateChange(RocketState state) {
    switch (state) {
        case RocketState::IDLE:
            idleState();
            break;
        case RocketState::IGNITION:
            ignitionState();
            break;
        case RocketState::ASCENT:
            ascentState();
            break;
        case RocketState::DESCENT:
            descentState();
            break;
        case RocketState::LANDING:
            landingState();
            break;
    }
}
