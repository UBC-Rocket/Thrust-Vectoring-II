#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>  // Include IMU library

// Define the pins for ignition control
#define BUTTON_PIN 12
#define PMOS_PIN 26
#define NMOS_PIN 25

// Global variable for state
enum class RocketState {
    IDLE,     // Waiting for ignition
    IGNITION, // Ignition sequence
    ASCENT,   // Ascent phase
    DESCENT,  // Descent phase
    LANDING   // Landing phase
};

// Current rocket state
RocketState currentState = RocketState::IDLE;

// Create IMU object
Adafruit_ICM20948 imu;

// Global variables for ignition states
bool pmosState = true;
bool nmosState = false;

void setup() {
    Serial.begin(115200);  // Initialize Serial Monitor
    while (!Serial) {}

    // Initialize the IMU
    if (!imu.begin_I2C()) {
        Serial.println("Failed to find IMU");
        while (true);  // Halt execution if IMU isn't found
    }
    Serial.println("IMU found");

    // Initialize Pins
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PMOS_PIN, OUTPUT);
    pinMode(NMOS_PIN, OUTPUT);

    // Transition to IDLE state initially
    currentState = RocketState::IDLE;
}

void loop() {
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
    // Wait for the button press to transition to IGNITION state
    if (digitalRead(BUTTON_PIN) == HIGH) {
        transitionTo(RocketState::IGNITION);
    }
}

void ignitionState() {
    Serial.println("In IGNITION state.");
    // Toggle PMOS and NMOS for ignition control
    digitalWrite(PMOS_PIN, !pmosState);
    digitalWrite(NMOS_PIN, !nmosState);
    pmosState = !pmosState;
    nmosState = !nmosState;

    // Wait for a short time before transitioning to ASCENT
    delay(2000);  // Adjust delay as needed
    transitionTo(RocketState::ASCENT);
}

void ascentState() {
    Serial.println("In ASCENT state.");
    // Collect IMU data (acceleration and gyroscope)
    sensors_event_t accel, gyro;
    imu.getEvent(&accel, &gyro, nullptr, nullptr);

    // Print IMU values for debugging
    Serial.print("Acceleration: ");
    Serial.print(accel.acceleration.x); Serial.print(", ");
    Serial.print(accel.acceleration.y); Serial.print(", ");
    Serial.println(accel.acceleration.z);

    // After some time or condition, transition to DESCENT
    transitionTo(RocketState::DESCENT);
}

void descentState() {
    Serial.println("In DESCENT state.");
    // Handle parachute deployment or other descent logic here
    // Transition to LANDING after a delay
    delay(5000);  // Adjust delay or conditions as needed
    transitionTo(RocketState::LANDING);
}

void landingState() {
    Serial.println("In LANDING state.");
    // Final data saving and reset system
    // Save flight data and perform any final actions
    transitionTo(RocketState::IDLE);  // Return to IDLE state
}

// Transition to the next state
void transitionTo(RocketState newState) {
    currentState = newState;
    handleStateChange(newState);
}

// Handle actions specific to each state
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
