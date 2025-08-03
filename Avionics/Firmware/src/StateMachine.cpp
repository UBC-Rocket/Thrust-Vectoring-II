// Firmware/src/StateMachine.cpp
#include "StateMachine.h"
#include "flightdata.h"
#include "PID_Control.h"

// Flight detection thresholds
const float LIFTOFF_ACCEL_THRESHOLD = 10.0;  // Minimum acceleration to confirm liftoff
const float BURNOUT_GRAVITY_THRESHOLD = -8.0;

const unsigned long RECOVERY_DURATION = 5000;

unsigned long phaseStartTime = 0;
bool parachuteDeployed = false;

const unsigned long MOTOR_BURN_DURATION = 3500; 
unsigned long ignitionStartTime = 0; 

extern FlightData currentData;
extern bool done;
extern FlightPhase currentPhase;

// Helper function to get string representation of flight phase
const char* getPhaseString(FlightPhase phase) {
  switch(phase) {
      case IDLE: return "IDLE";
      case IGNITION: return "IGNITION";
      case POWERED_FLIGHT: return "POWERED_FLIGHT";
      case COASTING: return "COASTING";
      case APOGEE: return "APOGEE";
      case RECOVERY: return "RECOVERY";
      case LANDED: return "LANDED";
      default: return "UNKNOWN";
  }
}


// Function to change flight phase with logging
void changeFlightPhase(FlightPhase newPhase) {
  // Skip if already in this phase
  if (currentPhase == newPhase) return;

  if (newPhase == IDLE) {
    ignitionStartTime = 0;  
  }

  bool validTransition = false;
  
  switch(currentPhase) {
    case IDLE:
      validTransition = (newPhase == IGNITION || newPhase == POWERED_FLIGHT);
      break;
    case IGNITION:
      validTransition = (newPhase == POWERED_FLIGHT || newPhase == IDLE);
      break;
    case POWERED_FLIGHT:
      validTransition = (newPhase == COASTING);
      break;
    case COASTING:
      validTransition = (newPhase == APOGEE);
      break;
    case APOGEE:
      validTransition = (newPhase == RECOVERY);
      break;
    case RECOVERY:
      validTransition = (newPhase == LANDED);
      break;
    case LANDED:
      validTransition = false;
      break;
    default:
      validTransition = false;
  }
  
  if (!validTransition) {
    Serial.print("INVALID STATE TRANSITION ATTEMPTED: ");
    Serial.print(getPhaseString(currentPhase));
    Serial.print(" -> ");
    Serial.println(getPhaseString(newPhase));
    return;
  }
  
  // Record previous state for logging
  FlightPhase previousPhase = currentPhase;
  
  // Update state and record transition time
  currentPhase = newPhase;
  phaseStartTime = millis();
  if (newPhase == IGNITION || newPhase == POWERED_FLIGHT) {
    // Only set ignitionStartTime once per flight
    if (ignitionStartTime == 0) {
        ignitionStartTime = phaseStartTime;
    }
  }
  
  // Log the transition with timestamps
  Serial.print(millis());
  Serial.print(",STATE_CHANGE,");
  Serial.print(getPhaseString(previousPhase));
  Serial.print("->");
  Serial.println(getPhaseString(newPhase));
}


void deployParachute() {
  if (!parachuteDeployed) {
      Serial.println("Deploying parachute...");
      callParachute();
      parachuteDeployed = true;
  }
}


void detectBurnout() {
  // Timer-based burnout detection from IGNITION start
  if (millis() - ignitionStartTime >= MOTOR_BURN_DURATION) {
      Serial.print("Motor burnout detected via timer. Total time since ignition: ");
      Serial.print(millis() - ignitionStartTime);
      Serial.print(" ms (threshold: ");
      Serial.print(MOTOR_BURN_DURATION);
      Serial.println(" ms)");
      changeFlightPhase(COASTING);
  }
}


void detectApogee() {
  Serial.print("Detecting Apogee");
  // Simply wait 0.5 second after entering coasting phase
  if (millis() - phaseStartTime >= 500) {  
      Serial.println("Apogee timer elapsed (0.5 second) - assuming apogee reached");
      changeFlightPhase(APOGEE);
  }
}


void waitToOpenParachute() {
  Serial.print("Currently in Apogee state");
  // Deploy parachute
  if (millis() - phaseStartTime >= 500) {  // 1 second
      Serial.println("Deploying parachute 0.5 second after apogee");
      deployParachute();
      changeFlightPhase(RECOVERY);
  }
}


void waitToLand() {
  Serial.print("Currently in Recovery state");
  // simpler time-based approach
  if (millis() - phaseStartTime >= RECOVERY_DURATION) {
      Serial.println("Recovery duration complete after 5 seconds free fall, assuming landed");
      changeFlightPhase(LANDED);
      done = true;  // Signal flight completion
  }
}


// Process state machine transitions
void processStateMachine() {
  switch(currentPhase) {
      case IDLE:
          prepParachute();
          // Wait for commands
          break;
          
      case POWERED_FLIGHT:
          // Handle burnout detection
          detectBurnout();
          break;
          
      case COASTING:
          // Handle apogee detection
          detectApogee();
          break;
          
      case APOGEE:
          waitToOpenParachute();
          break;
          
      case RECOVERY:
          waitToLand();
          break;
          
      case LANDED:
          Serial.print("Flight LANDED");
          // Flight complete - maintain this state
          break;
  }
}


// Initialize state machine
void initStateMachine() {
    // Set initial values
    currentPhase = IDLE;
    phaseStartTime = millis();
    parachuteDeployed = false;
}
