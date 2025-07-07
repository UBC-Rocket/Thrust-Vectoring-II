// Firmware/src/PID_Control.cpp

#include "PID_Control.h"
#include "IMU_Control.h"
#include "flightdata.h"
#include <math.h>
#include <ESP32Servo.h>

// Servo, PID Controller Constants and Variables for X and Y Axes
Servo gimbal_x;
Servo gimbal_y;
const double Kp = 0.297, Ki = 0.00155, Kd = 0.0569;
double setpointX = 0.0, inputX, outputX; // X-axis PID variables
double setpointY = 0.0, inputY, outputY; // Y-axis PID variables

// Initialize PID controllers for X and Y axes
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);


void PID_Config(){
    // Attach servos to GPIO pins with appropriate PWM parameters
    gimbal_x.attach(16, 850, 3000);
    gimbal_y.attach(17, 850, 3000);
    
    // Initialize PID controllers and set output limits for stabilization
    pidX.SetMode(AUTOMATIC);
    pidY.SetMode(AUTOMATIC);
    pidX.SetOutputLimits(-20, 20);
    pidY.SetOutputLimits(-20, 20);
}


double servoY_PWM(double gimbalY) {
  return 1453 - 39.3 * gimbalY + 0.464 * pow(gimbalY, 2);
}


double servoX_PWM(double gimbalX) {
  return 1373 + 48.6 * gimbalX - 0.288 * pow(gimbalX, 2);
}


void PID_Loop(){
    updateIMUWithKalman();

    currentData.update_values();

    // Update PID input values with Kalman filtered angles instead of raw gyro rates
    // Using filtered angles provides better stability than raw gyro rates
    inputX = getFilteredRoll(); // X-axis (pitch) stabilization using Kalman filtered angle
    Serial.print("X: ");
    Serial.println(outputX);

    if (abs(inputX) > 0.01) {
      pidX.Compute();
      gimbal_x.writeMicroseconds(servoX_PWM(-outputX));
    }

    
    inputY = getFilteredPitch(); // Y-axis (roll) stabilization using Kalman filtered angle
    Serial.print("Y: ");
    Serial.println(outputY);

    if (abs(inputY) > 0.01) {
      pidY.Compute();
      gimbal_y.writeMicroseconds(servoY_PWM(outputY));
    }
}