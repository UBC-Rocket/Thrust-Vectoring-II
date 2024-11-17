/*
* To download the data as a .csv file:
*     1. Wait for the flight to be finished (all data is collected)
*     2. Connect to the ESP32's WiFi network (name is TVR and password is 12345678) 
*     3. On a web browser go to http://192.168.4.1/download
*/

#include "main.h"
#include "flightdata.h"

// Definitions
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG 0x14
#define ICM_ADDR 0x68

const char* ssid = "TVR";
const char* password = "12345678";

Adafruit_ICM20948 imu;
WiFiServer wifiServer(80);
bool done = false; // whether or not flight is finished
String receivedMessage = "";

// For testing
Servo gimbal_x;
Servo gimbal_y;

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

  // Start the Wi-Fi server
  wifiServer.begin();
  Serial.println("Wi-Fi server started.");
  
  if (!imu.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("Failed to find imu20948 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("imu20948 found");

  // For testing
  gimbal_x.attach(16, 1000, 2000);
  gimbal_y.attach(17, 1000, 2000);

  if (!SPIFFS.begin(true)) {
      Serial.println("Failed to mount file system");
      return;
  }
  Serial.println("Mounted file system");

  setLowNoiseMode();

  Serial.println("Keep IMU still. Calibrating gyroscope and accelerometer...");

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
  imu.setAccelRateDivisor(5);

  calibrateGyroAccel();

  initialize_csv();
  startTime = millis();
}

void loop() {
  if (!done) {
    currentData.update_values();
    // currentData.print_values();
    currentData.save_values();
  }

  WiFiClient client = wifiServer.available();
  if (client) {
      Serial.println("Client connected");

      // Wait for a request from the client
      String request = client.readStringUntil('\r');
      client.flush();

      // Check if the request is for the CSV download
      if (request.indexOf("GET /download") != -1) {
          done = true;
          currentData.serve_csv(client);
      } else {
          // Send a 404 Not Found response if the request is not for /download
          client.println("HTTP/1.1 404 Not Found");
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.println();
          client.println("File not found");
      }

      // Close the connection
      client.stop();
      Serial.println("Client disconnected");
  }

  delay(10); // 10 ms delay = 100 Hz
}

// Set low noise modes for both gyroscope and accelerometer
void setLowNoiseMode() {
  imu.writeExternalRegister(ICM_ADDR, GYRO_CONFIG_1, 0x01);
  imu.writeExternalRegister(ICM_ADDR, ACCEL_CONFIG, 0x01);
}