/*
* To download the data as a .csv file:
*     1. Wait for the flight to be finished (all data is collected)
*     2. Connect to the ESP32's WiFi network (name is TVR and password is 12345678) 
*     3. On a web browser go to http://192.168.4.1/download
*/

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

void setup() {
  Wire.begin(21, 22); // SDA on GPIO 21, SCL on GPIO 22   
  Serial.begin(115200);
  while(!Serial) {}

  Serial.println("\n\n==========================================");
  Serial.println("STARTING WIFI INITIALIZATION");
  Serial.println("==========================================");
  bool wifiResult = initWifiAccessPoint();
  Serial.println("WiFi AP Initialization result: " + String(wifiResult ? "SUCCESS" : "FAILED"));
  
  Serial.println("Starting WiFi server...");
  bool serverResult = startWifiServer();
  Serial.println("WiFi server result: " + String(serverResult ? "SUCCESS" : "FAILED"));
  
  initIMU();
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

  initialize_csv();
  startTime = millis();
}

void loop() {
  if ((WAIT_FOR_EMATCH && started && !done) || (!WAIT_FOR_EMATCH && !done)) {
    PID_Loop();
  }

  // ======== Remote Ignition Control & Remote CSV Download (Wi-Fi Command Listening) ======== //
  remoteControl(beginFlight);

  yield();
  delay(10);
}

// Flip PMOS and NMOS states for ignition control, begin gimbal control and data logging
void beginFlight() {

  Serial.println("CONNECTED");
  started = true;
  pmosState = !pmosState;  // Toggle PMOS state
  nmosState = !nmosState;  // Toggle NMOS state
  digitalWrite(PMOS_PIN, pmosState);
  digitalWrite(NMOS_PIN, nmosState);

  // Debugging output to monitor PMOS and NMOS states
  // Serial.print("PMOS: ");
  // Serial.println(pmosState);
  // Serial.print("NMOS: ");
  // Serial.println(nmosState);
}