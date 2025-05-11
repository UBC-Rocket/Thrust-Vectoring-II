// Firmware/src/main.cpp

#include "main.h"

#include "Wifi_Control.h"
#include "IMU_Control.h"
#include "PID_Control.h"
#include "flightdata.h"

const bool WAIT_FOR_EMATCH = false; // set to true if this is a real launch/test - 
                                    // this will prevent data logging and servo movement until the ematch is lit
extern bool done; // Is flight finished
bool started = false; // Is flight started

// PMOS and NMOS pins for remote ignition control
const int PMOS_PIN = 26;
const int NMOS_PIN = 25;
bool pmosState = true;
bool nmosState = false;

unsigned long ignitionTime = 0;
const unsigned long IGNITION_DURATION = 1000; // 2 seconds for ignition
bool ignitionActive = false;

const unsigned long RECOVERY_DURATION = 15000;


enum FlightPhase {
  IDLE,               // Initial state - waiting for command
  IGNITION,           // Ignition command sent, waiting for thrust
  POWERED_FLIGHT,     // Motor burning, rocket accelerating
  COASTING,           // Motor burnout, still ascending
  APOGEE,             // Highest point reached, brief moment of near-zero acceleration
  RECOVERY, // Parachute deployment initiated
  LANDED              // On the ground, flight complete
};


// Flight detection thresholds
// These values are derived from expected flight parameters of this specific rocket
// Acceleration thresholds in m/s²
const float LIFTOFF_ACCEL_THRESHOLD = 12.0;  // Minimum acceleration to confirm liftoff
const float LIFTOFF_ACCEL_INCREMENT = 0.5;   // Minimum increment to count as acceleration increase
const float LIFTOFF_CONFIRM_COUNT = 3;       // Number of consecutive increases required

// Burnout detection parameters
const float BURNOUT_MAX_ACCEL_THRESHOLD = 15.0;  // Minimum peak acceleration during powered flight
const float BURNOUT_CURRENT_ACCEL_THRESHOLD = 12.0;  // Maximum current acceleration to consider burnout
const float BURNOUT_REDUCTION_FACTOR = 0.5;  // Minimum ratio of current/max acceleration

const unsigned long PARACHUTE_DEPLOY_CONFIRM_TIME = 1000; // ms
const unsigned long PARACHUTE_DEPLOY_TIMEOUT = 3000;     // ms maximum wait time



FlightPhase currentPhase = IDLE;
unsigned long phaseStartTime = 0;

bool parachuteDeployed = false;
const int PARACHUTE_PIN = 12;  // TODO set to actual pin


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
      // No valid transitions from LANDED
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


// Deploy parachute function
void deployParachute() {
  if (!parachuteDeployed) {
      Serial.println("Deploying parachute...");
      
      // In future implementation: Activate parachute deployment mechanism
      // For now, just print to serial
      // pinMode(PARACHUTE_PIN, OUTPUT);
      // digitalWrite(PARACHUTE_PIN, HIGH);
      
      // Set deployment flag
      parachuteDeployed = true;
      
      Serial.println("Parachute deployment signal sent");
  }
}


// Detection functions for state transitions
void detectIgnition() {
  // Calculate acceleration magnitude from all axes
  float accelMagnitude = sqrt(
      pow(currentData.getAccel().x, 2) + 
      pow(currentData.getAccel().y, 2) + 
      pow(currentData.getAccel().z, 2)
  );
  
  // Track if acceleration is increasing (sign of successful ignition)
  static float prevMaxAccel = 0;
  static int accelIncreaseCount = 0;
  
  if (accelMagnitude > prevMaxAccel + 0.5) {  // 0.5 m/s² threshold for increase
      accelIncreaseCount++;
      prevMaxAccel = accelMagnitude;
  }
  
  // Transition based on sufficient acceleration AND sustained increase
  if (accelMagnitude > LIFTOFF_ACCEL_THRESHOLD && accelIncreaseCount >= LIFTOFF_CONFIRM_COUNT) {
      Serial.print("Lift-off detected with acceleration: ");
      Serial.print(accelMagnitude);
      Serial.println(" m/s²");
      
      // Reset detection variables for future use
      prevMaxAccel = 0;
      accelIncreaseCount = 0;
      
      changeFlightPhase(POWERED_FLIGHT);
  }
  
  // Safety timeout for failed ignition
  if (!ignitionActive && millis() - phaseStartTime > 5000) {
      Serial.println("ERROR: No significant acceleration detected after ignition timeout");
      // For now, just go back to IDLE state
      changeFlightPhase(IDLE);
  }
}


void detectBurnout() {
  // Get vertical acceleration (assuming Z is vertical axis)
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
  
  // Minimum powered flight duration safety check
  if (millis() - phaseStartTime < 500) {
    // Skip burnout detection during first 500ms of powered flight
    return;
  }
  
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
  // Simply wait 3 seconds after entering coasting phase
  if (millis() - phaseStartTime >= 3000) {  // 3 seconds
      Serial.println("Apogee timer elapsed (8 seconds) - assuming apogee reached");
      changeFlightPhase(APOGEE);
  }
}


// Function to handle ignition circuit safety
void handleIgnitionSafety() {
  if (ignitionActive && (millis() - ignitionTime >= IGNITION_DURATION)) {
      // Turn off ignition circuit
      pmosState = true;   // OFF for PMOS
      nmosState = false;  // OFF for NMOS
      digitalWrite(PMOS_PIN, pmosState);
      digitalWrite(NMOS_PIN, nmosState);
      ignitionActive = false;
      Serial.println("Ignition circuit turned off automatically");
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
          // Deploy parachute
          if (millis() - phaseStartTime >= 3000) {  // 3 seconds
              Serial.println("Deploying parachute 3 seconds after apogee");
              deployParachute();
              changeFlightPhase(RECOVERY);
          }
          break;
          
          case RECOVERY:
            // simpler time-based approach
            if (millis() - phaseStartTime >= RECOVERY_DURATION) {
                Serial.println("Recovery duration complete, assuming landed");
                changeFlightPhase(LANDED);
                done = true;  // Signal flight completion
            }
            break;
          
      case LANDED:
          // Flight complete - maintain this state
          break;
  }
}


void setup() {
  Wire.begin(21, 22); // SDA on GPIO 21, SCL on GPIO 22   
  Serial.begin(115200);
  while(!Serial) {}

  Serial.println("STARTING WIFI INITIALIZATION");
  bool wifiResult = initWifiAccessPoint();
  Serial.println("WiFi AP Initialization result: " + String(wifiResult ? "SUCCESS" : "FAILED"));
  
  Serial.println("Starting WiFi server...");
  bool serverResult = startWifiServer();
  Serial.println("WiFi server result: " + String(serverResult ? "SUCCESS" : "FAILED"));
  
  if (!initIMU()) {
    // Handle initialization failure
    Serial.println("FATAL: Cannot continue without IMU");
    // halt
    while (true) {
      delay(100);
    }
  }
  
  PID_Config();

  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount SPIFFS. Formatting...");
    
    // Format SPIFFS
    if (SPIFFS.format()) {
      Serial.println("SPIFFS formatted successfully.");
    } else {
      Serial.println("Failed to format SPIFFS. Check partition configuration.");
      return; // Exit setup if formatting fails
    }
    
    // Attempt to mount SPIFFS again after formatting
    if (!SPIFFS.begin()) {
      Serial.println("Failed to mount SPIFFS after formatting.");
      return; // Exit setup if mounting still fails
    }
  }
  Serial.println("Mounted file system");

  configIMU();

  // Initialize PMOS and NMOS pins as outputs for ignition control
  pinMode(PMOS_PIN, OUTPUT);
  pinMode(NMOS_PIN, OUTPUT);

  // Set initial states for PMOS and NMOS
  digitalWrite(PMOS_PIN, pmosState);
  digitalWrite(NMOS_PIN, nmosState);

  if (!initialize_csv()) {
    Serial.println("WARNING: Data logging initialization failed");
  }
  startTime = millis();
}


void loop() {
  // Process state machine transitions first
  processStateMachine();
  
  // Apply appropriate control based on current state
  // Only run PID control during active flight phases that need it
  if (currentPhase == IGNITION || currentPhase == POWERED_FLIGHT || 
      currentPhase == COASTING) {
      PID_Loop();
  }
  
  // Process ignition circuit safety shutdown
  handleIgnitionSafety();
  
  // Handle communication and other background tasks
  remoteControl(beginFlight);
  yield();
  delay(10);
}


// Flip PMOS and NMOS states for ignition control, begin gimbal control and data logging
void beginFlight() {
  Serial.println("CONNECTED");
  
  if (currentPhase == IDLE) {
      // Change to IGNITION state
      changeFlightPhase(IGNITION);
      
      // Turn ignition circuit ON
      pmosState = false;  // ON for PMOS (active low)
      nmosState = true;   // ON for NMOS (active high)
      digitalWrite(PMOS_PIN, pmosState);
      digitalWrite(NMOS_PIN, nmosState);
      
      // Set ignition timestamp and flag
      ignitionTime = millis();
      ignitionActive = true;
      
      // Change to IGNITION state
      changeFlightPhase(IGNITION);
      started = true; // Maintain compatibility with existing code
      
      Serial.println("Ignition circuit activated");
  } else {
      Serial.println("Command ignored - rocket not in IDLE state");
  }
}