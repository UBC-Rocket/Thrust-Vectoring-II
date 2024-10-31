#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68
#include <ESP32Servo.h>
#include <WiFi.h>

// Wi-Fi credentials
const char *ssid = "TVR";
const char *password = "12345678";

// Wi-Fi server on port 80
WiFiServer wifiServer(80);

// Store the received command/message
String receivedMessage = "";

// ICM20948 IMU and Servo Setup
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
Servo gimbal_x;
Servo gimbal_y;

// PMOS and NMOS pins
const int PMOS_PIN = 26;  // Example pin for PMOS
const int NMOS_PIN = 25;  // Example pin for NMOS

// Current states of PMOS and NMOS
bool pmosState = true;
bool nmosState = false;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Initialize Wi-Fi as an Access Point
  Serial.println("Setting up Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP: ");
  Serial.println(IP);

  // Start the Wi-Fi server
  wifiServer.begin();
  Serial.println("Wi-Fi server started.");

  // Initialize IMU
  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  // Attach servos to pins
  gimbal_x.attach(16, 1000, 2000);
  gimbal_y.attach(17, 1000, 2000);

  // IMU calibration
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!");

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);

  // Initialize PMOS and NMOS pins as outputs
  pinMode(PMOS_PIN, OUTPUT);
  pinMode(NMOS_PIN, OUTPUT);

  // Set initial states for PMOS and NMOS
  digitalWrite(PMOS_PIN, pmosState);
  digitalWrite(NMOS_PIN, nmosState);
}

void flipPMOS_NMOS() {
  // Flip the states of PMOS and NMOS
  pmosState = !pmosState;
  nmosState = !nmosState;

  // Set the new states to the pins
  digitalWrite(PMOS_PIN, pmosState);
  digitalWrite(NMOS_PIN, nmosState);

  // Debug: Print the new states
  Serial.print("PMOS: ");
  Serial.println(pmosState);
  Serial.print("NMOS: ");
  Serial.println(nmosState);
}

void loop() {
  // Handle IMU data (optional - original logic)
  myIMU.readSensor();
  xyzFloat gValue = myIMU.getGValues();
  xyzFloat angle = myIMU.getAngles();

  
  gimbal_x.write(angle.x*5);
  gimbal_y.write(angle.y*5);

  // Check for new client connection
  WiFiClient client = wifiServer.available();
  if (client) {
    Serial.println("New Client Connected.");
    String currentLine = "";  // Buffer for incoming data

    // Read data from the client
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();  // Read each character

        // Debug: Print each character as it arrives
        Serial.print("Received character: ");
        Serial.println(c);

        // Check if the received character is 'A'
        if (c == 'A') {
          flipPMOS_NMOS();  // Call the flip function
        }
      }
    }

    client.stop();  // Close the connection
    Serial.println("Client Disconnected.");
  }
}
