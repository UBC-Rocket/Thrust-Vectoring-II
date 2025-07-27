// Firmware/src/PID_Control.cpp

#include "PID_Control.h"
#include "flightdata.h"

// Servo, PID Controller Constants and Variables for X and Y Axes
Servo gimbal_x;
Servo gimbal_y;
const int servoxinit = 60, servoyinit = 70; // Servo initial positions
const double Kp = 1, Ki = 2, Kd = 0;
double setpointX = 0.0, inputX, outputX; // X-axis PID variables
double setpointY = 0.0, inputY, outputY; // Y-axis PID variables

// Servo sweep parameters
static const int MIN = 0;
static const int MAX = 180;
static const int STEP_SIZE = 5;
static const unsigned long PAUSE_MS = 2000;

// Initialize PID controllers for X and Y axes
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

void PID_Config(){
  // Attach servos to GPIO pins with appropriate PWM parameters
  gimbal_x.attach(16, 1000, 2000);
  gimbal_y.attach(17, 1000, 2000);

  // Initialize PID controllers and set output limits for stabilization
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-20, 20);
  pidY.SetOutputLimits(-20, 20);
}

void PID_Loop(){
    currentData.update_values();
    currentData.save_values();

    // Update PID input values with current IMU data
    inputX = currentData.getGyro().x; // X-axis (pitch) stabilization
    if (abs(inputX) > 0.01) {
      pidX.Compute();   // Compute PID output for X-axis
      gimbal_x.write(servoxinit + outputX * SERVO_MULTIPLIER); // Adjust gimbal X servo
    }

    inputY = currentData.getGyro().y; // Y-axis (roll) stabilization
    if (abs(inputY) > 0.01) {
      pidY.Compute();   // Compute PID output for Y-axis
      gimbal_y.write(servoyinit + outputY * SERVO_MULTIPLIER); // Adjust gimbal Y servo
    }
}

void sweepServosOnce() {
  // only run once
  static bool done = false;
  if (done) return;
  done = true;

  // X axis forward and back
  for (int p = MIN; p <= MAX; p += STEP_SIZE) {
    gimbal_x.write(p);
    Serial.print("X: ");
    Serial.println(p);
    delay(PAUSE_MS);
  }
  for (int p = MAX; p >= MIN; p -= STEP_SIZE) {
    gimbal_x.write(p);
    Serial.print("X: ");
    Serial.println(p);
    delay(PAUSE_MS);
  }

  // Y axis forward and back
  for (int p = MIN; p <= MAX; p += STEP_SIZE) {
    gimbal_y.write(p);
    Serial.print("Y: ");
    Serial.println(p);
    delay(PAUSE_MS);
  }
  for (int p = MAX; p >= MIN; p -= STEP_SIZE) {
    gimbal_y.write(p);
    Serial.print("Y: ");
    Serial.println(p);
    delay(PAUSE_MS);
  }
}