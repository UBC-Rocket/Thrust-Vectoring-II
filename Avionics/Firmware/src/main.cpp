/*
* To download the data as a .csv file:
*     1. Wait for the flight to be finished (all data is collected)
*     2. Connect to the ESP32's WiFi network (name is TVR and password is 12345678) 
*     3. On a web browser go to http://192.168.4.1/download
*/

#include "main.h"
#include "flightdata.h"

#include "Wifi_Control.h"
#include "IMU_Control.h"
#include "PID_Control.h"

const bool WAIT_FOR_EMATCH = false; // set to true if this is a real launch/test - 
                                    // this will prevent data logging and servo movement until the ematch is lit
bool done = false; // Is flight finished
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

  initWifiAccessPoint();
  startWifiServer();
  
  initIMU();
  PID_Config();

  if (!SPIFFS.begin(true)) {
      Serial.println("Failed to mount file system");
      return;
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
}

// Flip PMOS and NMOS states for ignition control, begin gimbal control and data logging
void beginFlight() {
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