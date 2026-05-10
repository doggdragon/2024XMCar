#ifndef PINS_H
#define PINS_H

// ===== LED & Buzzer (active LOW) =====
#define PIN_LED      PB9
#define PIN_BUZZER   PB8

// ===== Buttons (active LOW, internal pull-up) =====
#define PIN_KEY1     PB4
#define PIN_KEY2     PB3

// ===== Motor direction =====
#define PIN_MOTOR_L_IN1  PA3
#define PIN_MOTOR_L_IN2  PA4
#define PIN_MOTOR_R_IN1  PA0
#define PIN_MOTOR_R_IN2  PA5

// ===== Motor PWM (TIM2, 20kHz) =====
#define PIN_MOTOR_L_PWM  PA2   // TIM2 CH3
#define PIN_MOTOR_R_PWM  PA1   // TIM2 CH2

// ===== Track sensors (active HIGH = line detected) =====
#define PIN_TRACK1  PA15
#define PIN_TRACK2  PA8
#define PIN_TRACK3  PA11
#define PIN_TRACK4  PB15
#define PIN_TRACK5  PA7
#define PIN_TRACK6  PB14
#define PIN_TRACK7  PA6
#define PIN_TRACK8  PB13

// ===== I2C: MPU6050 on PB6/PB7, OLED on PB10/PB11 =====
#define PIN_MPU_SCL   PB6
#define PIN_MPU_SDA   PB7
#define PIN_OLED_SCL  PB10
#define PIN_OLED_SDA  PB11

// ===== OLED display params =====
#define OLED_WIDTH   128
#define OLED_HEIGHT  32
#define OLED_ADDR    0x3C   // 7-bit (8-bit = 0x78)

// ===== PWM params =====
#define PWM_PERIOD   100    // ARR (0-99)
#define PWM_MAX      50
#define PWM_MIN     -50

#endif
