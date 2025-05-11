// Firmware/src/flightdata.cpp
#include "flightdata.h"
#include "main.h"
#include "IMU_Control.h"

extern Adafruit_ICM20948 imu;

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

sensors_vec_t FlightData::getMag() const {
    return magnetic;
}

float FlightData::getTemp() const {
    return temperature;
}


/*
* Should add a check here to know when the flight is over (set done = true).
* Possibly check if max accel has been greater than x and that accel has been < y for z number of checks.
* So that the csv file isn't unnecessarily massive with zeroes.
*/
void FlightData::update_values() {
  if (done) return;
  time = millis() - startTime;
  
  sensors_event_t accel, gyro, mag, temp;
  imu.getEvent(&accel, &gyro, &mag, &temp);

  this->magnetic = mag.magnetic;
  this->temperature = temp.temperature;

  // Account for IMU rotation within rocket:
  this->acceleration.z = accel.acceleration.x - accel_x_offset;
  this->acceleration.y = accel.acceleration.z - accel_z_offset;
  this->acceleration.x = accel.acceleration.y - accel_y_offset;
  this->gyroscope.z = gyro.gyro.x - gyro_x_offset;
  this->gyroscope.y = gyro.gyro.z - gyro_z_offset;
  this->gyroscope.x = gyro.gyro.y - gyro_y_offset;
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
    acceleration.x, acceleration.y, acceleration.z,
    gyroscope.x, gyroscope.y, gyroscope.z,
    magnetic.x, magnetic.y, magnetic.z,
    temperature, flightPhase);
    
  // Write the buffer to file
  if (bytesWritten > 0 && bytesWritten < sizeof(buffer)) {
    file.write(buffer, bytesWritten);
  } else {
    Serial.println("Error formatting CSV data");
  }
}


void initialize_csv() {
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

  bool headerWriteSuccess = true;
  headerWriteSuccess &= file.print("Time (ms)") && file.print(",");
  headerWriteSuccess &= file.print("Accel x (+/- 0.1 m/s^2)") && file.print(",");
  headerWriteSuccess &= file.print("Accel y (+/- 0.1 m/s^2)") && file.print(",");
  headerWriteSuccess &= file.print("Accel z (+/- 0.1 m/s^2)") && file.print(",");
  headerWriteSuccess &= file.print("Gyro x (+/- 0.2 rad/s)") && file.print(",");
  headerWriteSuccess &= file.print("Gyro y (+/- 0.2 rad/s)") && file.print(",");
  headerWriteSuccess &= file.print("Gyro z (+/- 0.2 rad/s)") && file.print(",");
  headerWriteSuccess &= file.print("Mag x (uT)") && file.print(",");
  headerWriteSuccess &= file.print("Mag y (uT)") && file.print(",");
  headerWriteSuccess &= file.print("Mag z (uT)") && file.print(",");
  headerWriteSuccess &= file.print("Temp (C)") && file.print(",");
  headerWriteSuccess &= file.println("Flight Phase");

  if (!headerWriteSuccess) {
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

