#include "motor.h"
#include "pins.h"

void Motor_Init() {
    // Direction pins
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);

    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);

    // PWM pins
    pinMode(PIN_MOTOR_L_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_PWM, OUTPUT);

    // PWM: use default frequency (~1kHz) for now
    // analogWriteFrequency(20000);  // DISABLED - may interfere with system

    // Start at 0 duty
    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);
}

void Set_Speed(int motor_l, int motor_r) {
    // Left motor direction
    if (motor_l >= 0) {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    } else {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        motor_l = -motor_l;
    }
    // Right motor direction
    if (motor_r >= 0) {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
    } else {
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        motor_r = -motor_r;
    }
    // Clamp to valid range
    if (motor_l > PWM_MAX) motor_l = PWM_MAX;
    if (motor_r > PWM_MAX) motor_r = PWM_MAX;

    // Map speed [0..50] to PWM [0..255]
    int pwm_l = motor_l * 255 / PWM_MAX;
    int pwm_r = motor_r * 255 / PWM_MAX;
    if (pwm_l > 255) pwm_l = 255;
    if (pwm_r > 255) pwm_r = 255;

    analogWrite(PIN_MOTOR_L_PWM, pwm_l);
    analogWrite(PIN_MOTOR_R_PWM, pwm_r);
}

void Limit(int *motor_left, int *motor_right) {
    if (*motor_left  > PWM_MAX) *motor_left  = PWM_MAX;
    if (*motor_left  < PWM_MIN) *motor_left  = PWM_MIN;
    if (*motor_right > PWM_MAX) *motor_right = PWM_MAX;
    if (*motor_right < PWM_MIN) *motor_right = PWM_MIN;
}
