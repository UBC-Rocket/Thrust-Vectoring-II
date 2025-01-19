#include "flightdata.h"
#include "main.h"

extern Adafruit_ICM20948 imu;

// Objects
File file;
float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;
float accel_x_offset = 0;
float accel_y_offset = 0;
float accel_z_offset = 0;
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

  accel.acceleration.x -= accel_x_offset;
  accel.acceleration.y -= accel_y_offset;
  accel.acceleration.z -= accel_z_offset;

  this->acceleration = accel.acceleration;

  gyro.gyro.x -= gyro_x_offset;
  gyro.gyro.y -= gyro_y_offset;
  gyro.gyro.z -= gyro_z_offset;
  
  this->gyroscope = gyro.gyro;
  this->magnetic = mag.magnetic;
  this->temperature = temp.temperature;
}

void FlightData::print_values() {
  Serial.println("\nTime: "); Serial.print(time);
  printVector(" - Accelerometer: ", acceleration);
  printVector("Gyroscope: ", gyroscope);  
  printVector("Magnetometer: ", magnetic);
  Serial.print("Temperature: "); Serial.print(temperature, 2);
}

void printVector(const char* label, sensors_vec_t vec) {
  Serial.print(label);
  Serial.print("x = "); Serial.print(vec.x, 2);
  Serial.print(", y = "); Serial.print(vec.y, 2);
  Serial.print(", z = "); Serial.print(vec.z, 2);
  Serial.println("\n");
}

void FlightData::save_values() {
  if (!file) {
    Serial.println("File is not open, can't save data");
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
      Serial.println("Failed to open file for writing");
      return;
  }
  Serial.println("Opened file for writing");

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
}

void calibrateGyroAccel() {
    int num_samples = 400;
    sensors_event_t gyro_event;
    sensors_event_t accel_event;
    sensors_event_t mag_event;  // placeholder
    sensors_event_t temp_event; // placeholder
    
    for (int i = 0; i < num_samples; i++) {
        imu.getEvent(&accel_event, &gyro_event, &mag_event, &temp_event);
        gyro_x_offset += gyro_event.gyro.x;
        gyro_y_offset += gyro_event.gyro.y;
        gyro_z_offset += gyro_event.gyro.z;
        accel_x_offset += accel_event.acceleration.x;
        accel_y_offset += accel_event.acceleration.y;
        accel_z_offset += accel_event.acceleration.z;
        delay(20);
    }
    gyro_x_offset /= num_samples;
    gyro_y_offset /= num_samples;
    gyro_z_offset /= num_samples;
    accel_x_offset /= num_samples;
    accel_y_offset /= num_samples;
    accel_z_offset /= num_samples;

    Serial.print("\nGyro offsets - X: "); Serial.print(gyro_x_offset);
    Serial.print(", Y: "); Serial.print(gyro_y_offset);
    Serial.print(", Z: "); Serial.println(gyro_z_offset);

    Serial.print("\nAccel offsets - X: "); Serial.print(accel_x_offset);
    Serial.print(", Y: "); Serial.print(accel_y_offset);
    Serial.print(", Z: "); Serial.println(accel_z_offset);
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

