// Firmware/src/main.cpp (modified)

#include "main.h"
#include "Wifi_Control.h"
#include "IMU_Control.h"
#include "PID_Control.h"
#include "flightdata.h"
#include "StateMachine.h" // Add the new header

extern bool done; // Is flight finished
bool started = false; // Is flight started

// PMOS and NMOS pins for remote ignition control
const int PMOS_PIN = 26;
const int NMOS_PIN = 25;
bool pmosState = true;
bool nmosState = false;

unsigned long ignitionTime = 0;
const unsigned long IGNITION_DURATION = 1000; // 1 second for ignition
bool ignitionActive = false;

FlightPhase currentPhase = IDLE;


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


void setup() {
  Wire.begin(21, 22); // SDA on GPIO 21, SCL on GPIO 22   
  Serial.begin(115200);
  while(!Serial) {
    delay(100);
  }

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

  // Initialize the state machine
  initStateMachine();

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
  if (currentPhase == IGNITION || currentPhase == POWERED_FLIGHT) {
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