# ICM20948 IMU Integration Documentation

## Overview
The ICM20948 is an integrated 9-axis motion tracking device incorporating:
- Gyroscope for angular velocity measurement
- Accelerometer for linear acceleration detection
- Magnetometer (AK09916) for magnetic field measurement

## Hardware Components

### Sensors Specifications

#### Gyroscope
- Function: Measures angular velocity
- Units: Degrees per second (DPS)
- Axes: X, Y, Z rotation
- Primary use: Stabilization control
- Available ranges: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s

#### Accelerometer
- Function: Measures linear acceleration
- Units: G-force or m/s²
- Axes: X, Y, Z directions
- Primary use: Tilt/orientation detection
- Available ranges: ±2g, ±4g, ±8g, ±16g

#### Magnetometer
- Function: Measures magnetic field strength
- Primary use: Digital compass/heading reference
- Capabilities: Acts like digital compass, determines absolute orientation
- Update rates: 10Hz, 20Hz, 50Hz, 100Hz

## Software Implementation

### Library Overview
The Adafruit_ICM20948 is a C++ class/library that provides an object-oriented interface for the ICM-20948 sensor, handling:
- I2C/SPI communication protocols
- Sensor configuration
- Data acquisition

### Function Documentation

#### begin_I2C()
```cpp
/**
* @brief Initializes I2C communication with the ICM-20948 IMU
* @param addr I2C address of IMU (default 0x68)
* @param wire TwoWire instance for I2C communication
* @return true if initialization successful, false otherwise
* @note Must be called before using any other IMU functions
* @example
* if (!imu.begin_I2C(ICM_ADDR, &Wire)) {
*   Serial.println("Failed to find ICM20948 chip");
* }
*/
```

#### setGyroRange()
```cpp
/**
* @brief Sets gyroscope measurement range/sensitivity
* @param range Gyro range - options:
*    ICM20948_GYRO_RANGE_250_DPS  (±250°/s)
*    ICM20948_GYRO_RANGE_500_DPS  (±500°/s)
*    ICM20948_GYRO_RANGE_1000_DPS (±1000°/s) 
*    ICM20948_GYRO_RANGE_2000_DPS (±2000°/s)
* @note Lower range = higher precision but can't measure fast rotation
* @example Used 250 DPS since rocket rotation < 250°/s
*/
```

#### setAccelRange()
```cpp
/**
* @brief Sets accelerometer measurement range/sensitivity
* @param range Accelerometer range - options:
*    ICM20948_ACCEL_RANGE_2_G  (±2g)
*    ICM20948_ACCEL_RANGE_4_G  (±4g)
*    ICM20948_ACCEL_RANGE_8_G  (±8g)
*    ICM20948_ACCEL_RANGE_16_G (±16g)
* @note Lower range = higher precision for slower movements
* @example Used 2G since rocket acceleration < 2g
*/
```

#### setMagDataRate()
```cpp
/**
* @brief Sets magnetometer data sampling rate
* @param rate Magnetometer update frequency - options:
*    AK09916_MAG_DATARATE_10_HZ
*    AK09916_MAG_DATARATE_20_HZ
*    AK09916_MAG_DATARATE_50_HZ
*    AK09916_MAG_DATARATE_100_HZ
* @note Higher rate = more frequent heading updates
* @example Used 100Hz for maximum update frequency
*/
```

#### setGyroRateDivisor()
```cpp
/**
* @brief Sets gyroscope output data rate divisor
* @param div Divides base rate (9kHz) by this value
* @note Final data rate = 9kHz/divisor
* @example div=9 results in 1kHz output rate
*/
```

#### setAccelRateDivisor()
```cpp
/**
* @brief Sets accelerometer output data rate divisor
* @param div Divides base rate by this value
* @note Final data rate = base_rate/divisor
* @example div=10 for balanced sampling rate
*/
```

#### writeExternalRegister()
```cpp
/**
* @brief Writes to IMU configuration registers
* @param addr Device I2C address (ICM_ADDR = 0x68)
* @param reg Register address (GYRO_CONFIG_1 = 0x01, ACCEL_CONFIG = 0x14)
* @param value Configuration value (0x01 = low noise mode)
* @note Used to set low noise mode for better precision
*/
```

#### getEvent()
```cpp
/**
* @brief Retrieves latest sensor data from IMU
* @return sensors_event_t struct containing:
*    - gyro (x,y,z rotation rates in rad/s)
*    - accel (x,y,z acceleration in m/s²)
*    - mag (x,y,z magnetic field in uT)
*    - temperature (°C)
* @note Called frequently to get latest motion data
*/
```

### Configuration Constants

#### Gyroscope Ranges
- `ICM20948_GYRO_RANGE_250_DPS`
- `ICM20948_GYRO_RANGE_500_DPS`
- `ICM20948_GYRO_RANGE_1000_DPS`
- `ICM20948_GYRO_RANGE_2000_DPS`

#### Accelerometer Ranges
- `ICM20948_ACCEL_RANGE_2_G`
- `ICM20948_ACCEL_RANGE_4_G`
- `ICM20948_ACCEL_RANGE_8_G`
- `ICM20948_ACCEL_RANGE_16_G`

#### Magnetometer Rates
- `AK09916_MAG_DATARATE_10_HZ`
- `AK09916_MAG_DATARATE_20_HZ`
- `AK09916_MAG_DATARATE_50_HZ`
- `AK09916_MAG_DATARATE_100_HZ`

## Performance Considerations

### Output Data Rate (ODR)
- Defines raw measurement frequency
- Affects sensor data output speed
- Impacts system responsiveness
- Base rates:
  - Gyroscope: 9kHz base rate
  - Accelerometer: Configurable base rate
  - Default I2C address: 0x68

### Sampling Configuration
When configuring sampling rates, consider:
- System responsiveness requirements
- Available processing power
- Required data resolution
- Power consumption constraints

### Current Implementation
- Gyroscope: 250 DPS range (rocket rotation < 250°/s)
- Accelerometer: 2G range (rocket acceleration < 2g)
- Magnetometer: 100Hz update rate (maximum frequency)