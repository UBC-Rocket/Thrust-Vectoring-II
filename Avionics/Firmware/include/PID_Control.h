// Firmware/include/PID_Control.h

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <PID_v1.h>
#include <ESP32Servo.h>

#define SERVO_MULTIPLIER 15

// Initialize servo motors, and servo offsets
extern Servo gimbal_x;
extern Servo gimbal_y;
extern const int servoxinit;
extern const int servoyinit;

extern PID pidX;
extern PID pidY;
extern double setpointX, inputX, outputX;
extern double setpointY, inputY, outputY;


void PID_Config();

void PID_Loop();

#endif //PID_CONTROL_H