// flightdata.cpp
#include "flightdata.h"
#include "main.h"
#include "IMU_Control.h"

extern Adafruit_ICM20948 imu;

// Objects
File file;

unsigned long startTime = 0;
FlightData currentData;

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
}

void FlightData::print_values() {
  // Serial.println("\nTime: "); Serial.print(time);
  // printVector(" - Accelerometer: ", acceleration);
  // printVector("Gyroscope: ", gyroscope);  
  // printVector("Magnetometer: ", magnetic);
  // Serial.print("Temperature: "); Serial.print(temperature, 2);
}

void FlightData::print_values_arduino() {
  Serial.print(acceleration.x);
  Serial.print("\t");
  Serial.print(acceleration.y);
  Serial.print("\t");
  Serial.println(acceleration.z);
}

void printVector(const char* label, sensors_vec_t vec) {
  // Serial.print(label);
  // Serial.print("x = "); Serial.print(vec.x, 2);
  // Serial.print(", y = "); Serial.print(vec.y, 2);
  // Serial.print(", z = "); Serial.print(vec.z, 2);
  // Serial.println("\n");
}

void FlightData::save_values() {
  if (!file) {
    // Serial.println("File is not open, can't save data");
    return;
  }

  String csvLine = String(time) + "," + String(acceleration.x, 2) + "," + String(acceleration.y, 2) + "," + String(acceleration.z, 2) + "," +
                 String(gyroscope.x, 2) + "," + String(gyroscope.y, 2) + "," + String(gyroscope.z, 2) + "," +
                 String(magnetic.x, 2) + "," + String(magnetic.y, 2) + "," + String(magnetic.z, 2) + "," + String(temperature, 2) + "\n";
  file.print(csvLine);
}

void initialize_csv() {
  file = SPIFFS.open("/data.csv", FILE_WRITE);
  if (!file) {
      Serial.println("Failed to open file for initializing. Formatting...");
      SPIFFS.format();
<<<<<<< HEAD
      if (!SPIFFS.begin()) {
        Serial.println("Failed to mount SPIFFS during formatting.");
        return;
      }
      
      // Try to open the file again after formatting
      file = SPIFFS.open("/data.csv", FILE_WRITE);
=======
      if (!SPIFFS.begin())
        Serial.println("Failed to mount SPIFFS during formatting.");
>>>>>>> e10866b (Axes correctly oriented)
      if (!file) {
        Serial.println("Failed to open file for initializing. Terminating...");
        return;
      }
  }
<<<<<<< HEAD
  
=======
>>>>>>> e10866b (Axes correctly oriented)
  Serial.println("Opened file for initializing");

  file.print("Time (ms)"); file.print(",");
  file.print("Accel x (+/- 0.1 m/s^2)"); file.print(",");
  file.print("Accel y (+/- 0.1 m/s^2)"); file.print(",");
  file.print("Accel z (+/- 0.1 m/s^2)"); file.print(",");
  file.print("Gyro x (+/- 0.2 rad/s)"); file.print(",");
  file.print("Gyro y (+/- 0.2 rad/s)"); file.print(",");
  file.print("Gyro z (+/- 0.2 rad/s)"); file.print(",");
  file.print("Mag x (uT)"); file.print(",");
  file.print("Mag y (uT)"); file.print(",");
  file.print("Mag z (uT)"); file.print(",");
  file.println("Temp (C)");

  file.close();
  file = SPIFFS.open("/data.csv", FILE_APPEND);
  Serial.println("Opened file for writing");
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

