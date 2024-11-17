/*
* To download the data as a .csv file:
*     1. Wait for the flight to be finished (all data is collected)
*     2. Connect to the ESP32's WiFi network (name is TVR and password is 12345678) 
*     3. On a web browser go to http://192.168.4.1/download
*/

#include "main.h"
#include "flightdata.h"

// Wi-Fi credentials
const char *ssid = "TVR";
const char *password = "12345678";

const int servoxinit = 60, servoyinit = 70; // Servo initial positions

// WiFi server setup
WiFiServer wifiServer(80);
String receivedMessage = "";
bool done = false; // Whether or not flight is finished

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
  Wire.begin(21, 22); // SDA on GPIO 21, SCL on GPIO 22   
  Serial.begin(115200);
  while(!Serial) {}

  // Initialize Wi-Fi as an Access Point
  Serial.println("Setting up Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP: ");
  Serial.println(IP);

  // Start the Wi-Fi server for remote ignition control and .csv upload
  wifiServer.begin();
  Serial.println("Wi-Fi server started.");
  
  // Initialize IMU
  if (!imu.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("Failed to find imu20948 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("imu20948 found");

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

  setLowNoiseMode();

  /* If rotation in any direction exceeds 250 degrees per second,
  *  then this value will have to be changed. 
  *  However, the lower the value is the more precise the measurement is. */ 
  imu.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);

  /* This rocket will be moving very slowly, 
  *  and acceleration will be far below the minimum range of double Earth's gravity.
  *  This means the highest precision is achieved. */ 
  imu.setAccelRange(ICM20948_ACCEL_RANGE_2_G);

  // Set magnetometer to update 100 times per second
  imu.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  // Set gyro and accel data rate divisors such that they update at 100Hz like the magnetometer
  imu.setGyroRateDivisor(90); // Gyroscope base output data rate is 9kHz
  imu.setAccelRateDivisor(10);

  Serial.println("Keep IMU still. Calibrating gyroscope and accelerometer...");
  calibrateGyroAccel();

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
  if (!done) {
    currentData.update_values();
    // currentData.print_values();
    currentData.save_values();

    // Update PID input values with current IMU data
    inputX = currentData.getGyro().x; // X-axis (pitch) stabilization
    pidX.Compute();   // Compute PID output for X-axis
    gimbal_x.write(servoxinit + outputX * 5); // Adjust gimbal X servo

    inputY = currentData.getGyro().y; // Y-axis (roll) stabilization
    pidY.Compute();   // Compute PID output for Y-axis
    gimbal_y.write(servoyinit + outputY * 5); // Adjust gimbal Y servo
  }

  // ======== Remote Ignition Control & Remote CSV Download (Wi-Fi Command Listening) ======== //
  WiFiClient client = wifiServer.available();
    if (client) {
        Serial.println("Client connected");

        String request = "";  // To store the HTTP request
        String currentLine = "";  // Buffer for incoming data

        // Wait for a request from the client
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();  // Read each character
                request += c;  // Build the request string
                
                // Debugging output for each received character
                Serial.print("Received character: ");
                Serial.println(c);

                // Check for the end of the line (indicates end of HTTP header)
                if (c == '\n') {
                    if (currentLine.length() == 0) {
                        // End of client request, process it
                        if (request.indexOf("GET /download") != -1) {
                            done = true;
                            currentData.serve_csv(client);  // Serve the CSV file
                        } else {
                            // Send a 404 Not Found response if the request is not for /download
                            client.println("HTTP/1.1 404 Not Found");
                            client.println("Content-Type: text/plain");
                            client.println("Connection: close");
                            client.println();
                            client.println("File not found");
                        }
                        break;  // Exit the loop after handling the request
                    } else {
                        // Reset current line for next header line
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    // Add characters to the current line (skip '\r')
                    currentLine += c;
                }

                // Check if the received character is 'A' (outside HTTP handling)
                if (c == 'A') {
                    flipPMOS_NMOS();  // Toggle PMOS and NMOS state
                }
            }
        }

        // Close the connection
        client.stop();
        Serial.println("Client disconnected");
    }

  delay(10); // 10 ms delay = 100 Hz
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

// Set low noise modes for both gyroscope and accelerometer
void setLowNoiseMode() {
  imu.writeExternalRegister(ICM_ADDR, GYRO_CONFIG_1, 0x01);
  imu.writeExternalRegister(ICM_ADDR, ACCEL_CONFIG, 0x01);
}