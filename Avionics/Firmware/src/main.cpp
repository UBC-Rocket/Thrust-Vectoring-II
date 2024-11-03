#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PID_v1.h>

// Define ICM20948 I2C Address and Wi-Fi credentials
#define ICM20948_ADDR 0x68
const char *ssid = "TVR";
const char *password = "12345678";

// Wi-Fi server setup
WiFiServer wifiServer(80);
String receivedMessage = ""; // Store received messages

// Initialize IMU (ICM20948), Servo Motors, and Servo Offsets
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
Servo gimbal_x;
Servo gimbal_y;
const int servoxinit = 60, servoyinit = 70; // Servo initial positions

// PID Controller Constants and Variables for X and Y Axes
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
  // Setup serial communication and I2C bus
  Wire.begin();
  Serial.begin(115200);

  // Configure Wi-Fi Access Point
  Serial.println("Setting up Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP: ");
  Serial.println(IP);
  
  // Start Wi-Fi server for remote ignition control
  wifiServer.begin();
  Serial.println("Wi-Fi server started.");

  // Initialize and calibrate IMU
  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  // Attach servos to GPIO pins with appropriate PWM parameters
  gimbal_x.attach(16, 1000, 2000);
  gimbal_y.attach(17, 1000, 2000);

  // Initialize PID controllers and set output limits for stabilization
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-20, 20);
  pidY.SetOutputLimits(-20, 20);

  // Calibrate IMU to set zero offsets
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!");

  // Configure IMU settings for accelerometer sensitivity and filter
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);

  // Initialize PMOS and NMOS pins as outputs for ignition control
  pinMode(PMOS_PIN, OUTPUT);
  pinMode(NMOS_PIN, OUTPUT);

  // Set initial states for PMOS and NMOS
  digitalWrite(PMOS_PIN, pmosState);
  digitalWrite(NMOS_PIN, nmosState);
}

// Flip PMOS and NMOS states for ignition control
void flipPMOS_NMOS() {
  pmosState = !pmosState;  // Toggle PMOS state
  nmosState = !nmosState;  // Toggle NMOS state
  digitalWrite(PMOS_PIN, pmosState);
  digitalWrite(NMOS_PIN, nmosState);

  // Debugging output to monitor PMOS and NMOS states
  Serial.print("PMOS: ");
  Serial.println(pmosState);
  Serial.print("NMOS: ");
  Serial.println(nmosState);
}

void loop() {
  // ======== IMU Data Reading and PID Control for Stabilization ========
  // Read sensor data from IMU
  myIMU.readSensor();
  xyzFloat angle = myIMU.getAngles(); // Get current pitch and roll angles

  // Update PID input values with current IMU data
  inputX = angle.x; // X-axis (pitch) stabilization
  pidX.Compute();   // Compute PID output for X-axis
  gimbal_x.write(servoxinit + outputX * 5); // Adjust gimbal X servo

  inputY = angle.y; // Y-axis (roll) stabilization
  pidY.Compute();   // Compute PID output for Y-axis
  gimbal_y.write(servoyinit + outputY * 5); // Adjust gimbal Y servo

  // ======== Remote Ignition Control (Wi-Fi Command Listening) ========
  // Check for new client connection for remote ignition
  WiFiClient client = wifiServer.available();
  if (client) {
    Serial.println("New Client Connected.");
    String currentLine = ""; // Buffer for incoming data

    // Read data from the client connection
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();  // Read each character

        // Debugging output for each received character
        Serial.print("Received character: ");
        Serial.println(c);

        // Check if the received character is 'A' to trigger ignition
        if (c == 'A') {
          flipPMOS_NMOS();  // Toggle PMOS and NMOS state
        }
      }
    }

    // Close the client connection once data is processed
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
