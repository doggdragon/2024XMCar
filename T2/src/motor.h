#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void Motor_Init();
void Set_Speed(int motor_l, int motor_r);
void Limit(int *motor_left, int *motor_right);

#endif
