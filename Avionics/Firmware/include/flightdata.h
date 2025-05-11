// Firmware/include/flightdata.h
#ifndef FLIGHTDATA_H
#define FLIGHTDATA_H

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <ESPAsyncWebServer.h>

/*
 * Data storage class:
 * - Holds most recent data from the IMU.
 * - Provides functions for printing and saving data.
 * - Provides functions for accessing the most recent data.
 */
class FlightData
{
    private:
        sensors_vec_t acceleration, gyroscope, magnetic;
        float temperature;
        unsigned long time;

    public:
        FlightData();

        sensors_vec_t getAccel() const;
        sensors_vec_t getGyro() const;
        sensors_vec_t getMag() const;
        float getTemp() const;
        
        void update_values();
        void print_values_arduino();
        void save_values();
        void serve_csv(WiFiClient& client);
        int flightPhase;
};

// FlightData object used to store current data and access it
extern FlightData currentData;

extern unsigned long startTime;

bool initialize_csv();

#endif // FLIGHTDATA_H
