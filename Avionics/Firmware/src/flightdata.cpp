// Firmware/src/flightdata.cpp
#include "flightdata.h"
#include "main.h"
#include "IMU_Control.h"
#include "PID_Control.h"

// Objects
File file;

unsigned long startTime = 0;
FlightData currentData;
extern bool done; // Declared in Wifi_Control.cpp


FlightData::FlightData() {
    acceleration = {0.0, 0.0, 0.0};
    gyroscope = {0.0, 0.0, 0.0};
    magnetic = {0.0, 0.0, 0.0};
    temperature = 0.0;
    time = 0;
}


sensors_vec_t FlightData::getAccel() const {
    return acceleration;
}


sensors_vec_t FlightData::getGyro() const {
    return gyroscope;
}


/*
* Should add a check here to know when the flight is over (set done = true).
* Possibly check if max accel has been greater than x and that accel has been < y for z number of checks.
* So that the csv file isn't unnecessarily massive with zeroes.
*/
void FlightData::update_values() {
    if (done) return;
    time = millis() - startTime;

    // Get calibrated sensor data from the new IMU control system
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    getCalibratedAcceleration(accel_x, accel_y, accel_z);
    getCalibratedGyroscope(gyro_x, gyro_y, gyro_z);

    // Account for IMU rotation within rocket (maintain existing coordinate transformation):
    this->acceleration.z = accel_x;  // Forward/backward becomes up/down
    this->acceleration.y = accel_z;  // Up/down becomes left/right
    this->acceleration.x = accel_y;  // Left/right becomes forward/backward

    this->gyroscope.z = gyro_x;
    this->gyroscope.y = gyro_z;
    this->gyroscope.x = gyro_y;

    flightPhase = (int)currentPhase;
}


void FlightData::save_values() {
    if (done) return;

    if (!file) {
        Serial.println("File is not open, can't save data");
        return;
    }

    // Pre-allocate a buffer for CSV formatting
    char buffer[256]; // Large enough for one CSV line

    // Format the entire line at once using snprintf
    int bytesWritten = snprintf(buffer, sizeof(buffer),
        "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
        time,
        inputX, outputX, inputY, outputY,
        acceleration.x, acceleration.y, acceleration.z,
        gyroscope.x, gyroscope.y, gyroscope.z,
        flightPhase);

    // Write the buffer to file
    if (bytesWritten > 0 && bytesWritten < sizeof(buffer)) {
        file.write((const uint8_t*)buffer, bytesWritten);
    } else {
        Serial.println("Error formatting CSV data");
    }
}


bool initialize_csv() {
  file = SPIFFS.open("/data.csv", FILE_WRITE);
  if (!file) {
      Serial.println("Failed to open file for initializing. Formatting...");
      SPIFFS.format();

      if (!SPIFFS.begin()) {
        Serial.println("Failed to mount SPIFFS during formatting.");
        return false;
      }
      
      // Try to open the file again after formatting
      file = SPIFFS.open("/data.csv", FILE_WRITE);

      if (!file) {
        Serial.println("Failed to open file for initializing. Terminating...");
        return false;
      }
  }

  Serial.println("Opened file for initializing");

  String header = "Time (ms),Accel x (+/- 0.1 m/s^2),Accel y (+/- 0.1 m/s^2),Accel z (+/- 0.1 m/s^2),";
  header += "Input x,Output x,Input y,Output y";
  header += "Gyro x (+/- 0.2 rad/s),Gyro y (+/- 0.2 rad/s),Gyro z (+/- 0.2 rad/s),Flight Phase";

  size_t bytesWritten = file.println(header);

  if (bytesWritten == 0) {
    Serial.println("Failed to write CSV header");
    file.close();
    return false;
  }

  file.close();
  file = SPIFFS.open("/data.csv", FILE_APPEND);
  
  if (!file) {
    Serial.println("Failed to reopen file for writing");
    return false;
  }
  
  Serial.println("Opened file for writing");
  return true;
}


void FlightData::serve_csv(WiFiClient& client) {
    if (file) {
        file.flush();
        file.close();
    }

    file = SPIFFS.open("/data.csv", FILE_READ);
    if (!file) {
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.println("Failed to open CSV file.");
        return;
    }

    // Send HTTP headers
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Connection: close");
    client.println();

    // Send file content
    while (file.available()) {
        client.write(file.read());
    }
    file.close();
}

