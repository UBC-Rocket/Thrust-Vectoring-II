// Firmware/src/StateMachine.cpp
#include "StateMachine.h"
#include "flightdata.h"

// Flight detection thresholds
const float LIFTOFF_ACCEL_THRESHOLD = 10.0;  // Minimum acceleration to confirm liftoff

// Burnout detection parameters
const float BURNOUT_MAX_ACCEL_THRESHOLD = 50.0;  // Minimum peak acceleration during powered flight
const float BURNOUT_CURRENT_ACCEL_THRESHOLD = 12.0;  // Maximum current acceleration to consider burnout
const float BURNOUT_REDUCTION_FACTOR = 0.5;  // Minimum ratio of current/max acceleration

const unsigned long RECOVERY_DURATION = 30000;

unsigned long phaseStartTime = 0;
bool parachuteDeployed = false;

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

  bool validTransition = false;
  
  switch(currentPhase) {
    case IDLE:
      validTransition = (newPhase == IGNITION);
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
      parachuteDeployed = true;
  }
}


// Detection functions for state transitions
void detectIgnition() {
  Serial.print("Detecting Ignition");
  // Calculate acceleration magnitude from all axes
  float accelMagnitude = sqrt(
      pow(currentData.getAccel().x, 2) + 
      pow(currentData.getAccel().y, 2) + 
      pow(currentData.getAccel().z, 2)
  );
  
  // Transition based on sufficient acceleration AND sustained increase
  if (accelMagnitude > LIFTOFF_ACCEL_THRESHOLD) {
      Serial.print("Lift-off detected with acceleration: ");
      Serial.print(accelMagnitude);
      Serial.println(" m/s²");
      
      changeFlightPhase(POWERED_FLIGHT);
  }
  
  // Safety timeout for failed ignition (5 seconds if not sufficient acceleration, go back to idle)
  if (millis() - phaseStartTime > 5000) {
      Serial.println("ERROR: No significant acceleration detected after ignition timeout");
      // For now, just go back to IDLE state
      changeFlightPhase(IDLE);
  }
}


void detectBurnout() {
  Serial.print("Detecting Burnout");
  // Get vertical acceleration
  float verticalAccel = currentData.getAccel().z;
  
  // Use a small window to smooth noise
  static float accelWindow[5] = {0};
  static int windowIndex = 0;
  
  // Update window with newest reading
  accelWindow[windowIndex] = verticalAccel;
  windowIndex = (windowIndex + 1) % 5;
  
  // Calculate average vertical acceleration
  float avgVertAccel = 0;
  for (int i = 0; i < 5; i++) {
    avgVertAccel += accelWindow[i];
  }
  avgVertAccel /= 5.0;
  
  // Check if acceleration has transitioned to gravity-dominated
  // -8.0 m/s² allows for some noise/calibration error but is clearly gravity-dominated
  if (avgVertAccel < -8.0) {
    Serial.print("Motor burnout detected. Vertical acceleration: ");
    Serial.print(avgVertAccel);
    Serial.println(" m/s² (gravity-dominated)");
    
    // Reset detection variables
    windowIndex = 0;
    for (int i = 0; i < 5; i++) {
      accelWindow[i] = 0;
    }
    
    changeFlightPhase(COASTING);
  }
}


void detectApogee() {
  Serial.print("Detecting Apogee");
  // Simply wait 3 seconds after entering coasting phase
  if (millis() - phaseStartTime >= 3000) {  // 3 seconds
      Serial.println("Apogee timer elapsed (3 seconds) - assuming apogee reached");
      changeFlightPhase(APOGEE);
  }
}


void waitToOpenParachute() {
  Serial.print("Currently in Apogee state");
  // Deploy parachute
  if (millis() - phaseStartTime >= 3000) {  // 3 seconds
      Serial.println("Deploying parachute 3 seconds after apogee");
      deployParachute();
      changeFlightPhase(RECOVERY);
  }
}


void waitToLand() {
  Serial.print("Currently in Recovery state");
  // simpler time-based approach
  if (millis() - phaseStartTime >= RECOVERY_DURATION) {
      Serial.println("Recovery duration complete after 30 seconds free fall, assuming landed");
      changeFlightPhase(LANDED);
      done = true;  // Signal flight completion
  }
}


// Process state machine transitions
void processStateMachine() {
  switch(currentPhase) {
      case IDLE:
          // Wait for commands
          break;
          
      case IGNITION:
          // Handle ignition detection
          detectIgnition();
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
          Serial.print("Currently in Landed state, flight COMPLETE");
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