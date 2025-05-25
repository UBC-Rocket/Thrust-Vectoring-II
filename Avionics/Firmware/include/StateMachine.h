// Firmware/include/StateMachine.h
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "main.h"
#include "flightdata.h"

// Flight detection thresholds
// These values are derived from expected flight parameters of this specific rocket
// Acceleration thresholds in m/s²
extern const float LIFTOFF_ACCEL_THRESHOLD;  // Minimum acceleration to confirm liftoff

extern const float BURNOUT_GRAVITY_THRESHOLD;

extern const unsigned long RECOVERY_DURATION;

extern unsigned long phaseStartTime;
extern bool parachuteDeployed;

// Helper function to get string representation of flight phase
const char* getPhaseString(FlightPhase phase);

// Function to change flight phase with logging
void changeFlightPhase(FlightPhase newPhase);

// Function to deploy parachute
void deployParachute();

// Detection functions for state transitions
void detectIgnition();
void detectBurnout();
void detectApogee();
void waitToOpenParachute();
void waitToLand();

// Process state machine transitions
void processStateMachine();

// Initialize state machine
void initStateMachine();

#endif // STATE_MACHINE_H