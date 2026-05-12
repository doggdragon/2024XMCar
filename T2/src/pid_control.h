#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

float Track_err();
int   PID_out(float error, int target);
int   pid_angle2(int target, int yaw);
void  Final_Speed(int pid_out, int base_speed);

#endif
