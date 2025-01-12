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

const bool WAIT_FOR_EMATCH = false; // set to true if this is a real launch/test - 
                                    // this will prevent data logging and servo movement until the ematch is lit

bool done = false; // Is flight finished
bool started = false; // Is flight started







/////////////////////////////////////////////////////////////////////////////////////
// Servo, PID Controller Constants and Variables for X and Y Axes
Servo gimbal_x;
Servo gimbal_y;
const int servoxinit = 60, servoyinit = 70; // Servo initial positions
const double Kp = 1, Ki = 0, Kd = 0;
double setpointX = 0.0, inputX, outputX; // X-axis PID variables
double setpointY = 0.0, inputY, outputY; // Y-axis PID variables

// Initialize PID controllers for X and Y axes
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);









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

////////////////////////////////////////////////////////////////////////////////////
  // Attach servos to GPIO pins with appropriate PWM parameters
  gimbal_x.attach(16, 1000, 2000);
  gimbal_y.attach(17, 1000, 2000);

  // Initialize PID controllers and set output limits for stabilization
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-20, 20);
  pidY.SetOutputLimits(-20, 20);










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
    currentData.update_values();
    // currentData.print_values();
    currentData.save_values();

    // Update PID input values with current IMU data
    inputX = currentData.getGyro().x; // X-axis (pitch) stabilization
    if (abs(inputX) > 0.01) {
      pidX.Compute();   // Compute PID output for X-axis
      gimbal_x.write(servoxinit + outputX * SERVO_MULTIPLIER); // Adjust gimbal X servo
    }

    inputY = currentData.getGyro().y; // Y-axis (roll) stabilization
    if (abs(inputY) > 0.01) {
      pidY.Compute();   // Compute PID output for Y-axis
      gimbal_y.write(servoyinit + outputY * SERVO_MULTIPLIER); // Adjust gimbal Y servo
    }
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