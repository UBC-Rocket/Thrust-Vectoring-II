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

const double Kp_x = 0.7539*2, Ki_x = 0.7231*1.1, Kd_x = 0.1965;
const double Kp_y = 0.8703*2, Ki_y = 3.3253*1.1, Kd_y = 0.0569;

double setpointX = 0.0, inputX, outputX; // X-axis PID variables
double setpointY = 0.0, inputY, outputY; // Y-axis PID variables

// Initialize PID controllers for X and Y axes
PID pidX(&inputX, &outputX, &setpointX, Kp_x, Ki_x, Kd_x, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp_y, Ki_y, Kd_y, DIRECT);


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
  double x_pwm = 1443 - 63.3 * gimbalX - 1.15 * pow(gimbalX, 2);
  if (x_pwm < 1000) {
    x_pwm = 1000;
  }
  else if (x_pwm > 2000) {
    x_pwm = 2000;
  }
  Serial.print("X PWM:");
  Serial.println(x_pwm);
  return x_pwm;
}

double servoY_PWM(double gimbalY) {
  double y_pwm = 1629 +78.4* gimbalY + 1.4* pow(gimbalY, 2);
  if (y_pwm < 1000) {
    y_pwm = 1000;
  }
  else if (y_pwm > 2000) {
    y_pwm = 2000;
  }
  Serial.print("Y PWM:");
  Serial.println(y_pwm);
  return y_pwm;
}

void PID_Loop(){

    updateIMUWithKalman();

    currentData.update_values();

    // Update PID input values with Kalman filtered angles instead of raw gyro rates
    // Using filtered angles provides better stability than raw gyro rates
    inputX = getFilteredRoll(); // X-axis (pitch) stabilization using Kalman filtered angle
    Serial.print("inX: ");
    Serial.println(inputX);
    Serial.print("X: ");
    Serial.println(outputX);

    pidX.Compute();
    gimbal_x.writeMicroseconds(servoX_PWM(outputX));
    
    inputY = getFilteredPitch(); // Y-axis (roll) stabilization using Kalman filtered angle
    Serial.print("inY: ");
    Serial.println(inputY);
    Serial.print("Y: ");
    Serial.println(outputY);

    pidY.Compute();
    gimbal_y.writeMicroseconds(servoY_PWM(-outputY)); 
    
}

void prepParachute(){
  parachute.write(-90);
}

void callParachute(){
  parachute.write(180);
}

