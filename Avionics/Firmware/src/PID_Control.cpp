// Firmware/src/PID_Control.cpp

#include "PID_Control.h"
#include "IMU_Control.h"
#include "flightdata.h"
#include <math.h>
#include <ESP32Servo.h>

// Servo, PID Controller Constants and Variables for X and Y Axes
Servo gimbal_x;
Servo gimbal_y;
Servo parachute;
// const double Kp = 0.03214912280701755, Ki = 0.02531390291806959, Kd = 0.1372680322128851;  TODO uncomment later
const double Kp = 3, Ki = 0, Kd = 0;
double setpointX = 0.0, inputX, outputX; // X-axis PID variables
double setpointY = 0.0, inputY, outputY; // Y-axis PID variables

// Initialize PID controllers for X and Y axes
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);


void PID_Config(){
    // Attach servos to GPIO pins with appropriate PWM parameters
    gimbal_x.attach(16, 850, 3000);
    gimbal_y.attach(17, 850, 3000);
    parachute.attach(18, 850, 3000);
    
    // Initialize PID controllers and set output limits for stabilization
    pidX.SetMode(AUTOMATIC);
    pidY.SetMode(AUTOMATIC);
    pidX.SetOutputLimits(-20, 20);
    pidY.SetOutputLimits(-20, 20);
}

double servoX_PWM(double gimbalX) {
  return 1443 - 63.3 * gimbalX - 1.15 * pow(gimbalX, 2);
}

double servoY_PWM(double gimbalY) {
  return 1142 + 32.5 * gimbalY + 3.27 * pow(gimbalY, 2);
}

void PID_Loop(){
    updateIMUWithKalman();

    currentData.update_values();

    // Update PID input values with Kalman filtered angles instead of raw gyro rates
    // Using filtered angles provides better stability than raw gyro rates
    inputX = getFilteredRoll(); // X-axis (pitch) stabilization using Kalman filtered angle
    Serial.print("X: ");
    Serial.println(outputX);

    pidX.Compute();
    gimbal_x.writeMicroseconds(servoX_PWM(-outputX));
    
    inputY = getFilteredPitch(); // Y-axis (roll) stabilization using Kalman filtered angle
    Serial.print("Y: ");
    Serial.println(outputY);

    pidY.Compute();
    gimbal_y.writeMicroseconds(servoY_PWM(outputY));
    
}

void prepParachute(){
  parachute.write(-90);
}

void callParachute(){
  parachute.write(180);
}

