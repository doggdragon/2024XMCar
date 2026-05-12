#include "pid_control.h"
#include "track_sensor.h"
#include "motor.h"

// ===== PID constants =====
#define Kp     2.5f
#define Kd     3.5f
#define KA2P   0.5f
#define KA2D   3.5f

static float last_error_val = 0;
static uint8_t last_ir_state = 0;

float Track_err() {
    uint8_t state = Get_Infrared_State();
    static uint8_t same_cnt = 0;

    if (state == last_ir_state) {
        same_cnt++;
    } else {
        same_cnt = 0;
        last_ir_state = state;
    }

    if (same_cnt < 2) {
        return last_error_val;
    }

    switch (state) {
        case 0x00: last_error_val =  0; break;  // 0000 0000
        case 0x10: last_error_val =  1; break;  // 0001 0000
        case 0x08: last_error_val =  1; break;  // 0000 1000
        case 0x18: last_error_val =  0; break;  // 0001 1000
        case 0x3C: last_error_val =  0; break;  // 0011 1100
        case 0x7E: last_error_val =  0; break;  // 0111 1110
        case 0x30: last_error_val =  2; break;  // 0011 0000
        case 0x20: last_error_val =  2; break;  // 0010 0000
        case 0x40: last_error_val =  4; break;  // 0100 0000
        case 0x60: last_error_val =  4; break;  // 0110 0000
        case 0x80: last_error_val =  6; break;  // 1000 0000
        case 0xC0: last_error_val =  6; break;  // 1100 0000
        case 0xE0: last_error_val =  6; break;  // 1110 0000
        case 0xA0: last_error_val =  6; break;  // 1010 0000
        case 0xFE: last_error_val =  6; break;  // 1111 1110
        case 0xFC: last_error_val =  6; break;  // 1111 1100
        case 0xF8: last_error_val =  6; break;  // 1111 1000
        case 0xF0: last_error_val =  6; break;  // 1111 0000
        case 0x7C: last_error_val =  4; break;  // 0111 1100
        case 0x78: last_error_val =  4; break;  // 0111 1000
        case 0x38: last_error_val =  2; break;  // 0011 1000
        case 0x0C: last_error_val = -2; break;  // 0000 1100
        case 0x0E: last_error_val = -4; break;  // 0000 1110
        case 0x1E: last_error_val = -4; break;  // 0001 1110
        case 0x3E: last_error_val = -4; break;  // 0011 1110
        case 0x1F: last_error_val = -6; break;  // 0001 1111
        case 0x3F: last_error_val = -6; break;  // 0011 1111
        case 0x7D: last_error_val = -6; break;  // 0111 1101
        case 0x04: last_error_val = -2; break;  // 0000 0100
        case 0x1C: last_error_val = -2; break;  // 0001 1100
        case 0x02: last_error_val = -4; break;  // 0000 0010
        case 0x06: last_error_val = -4; break;  // 0000 0110
        case 0x01: last_error_val = -6; break;  // 0000 0001
        case 0x03: last_error_val = -6; break;  // 0000 0011
        case 0x07: last_error_val = -6; break;  // 0000 0111
        case 0x0F: last_error_val = -6; break;  // 0000 1111
        default: break;
    }

    last_ir_state = state;
    same_cnt = 0;
    return last_error_val;
}

int PID_out(float error, int target) {
    (void)target;  // unused
    static int last_err = 0;
    int err = (int)error;

    int diff = err - last_err;
    if (last_err == 0 && err != 0) {
        diff = err / 2;
    }

    int out = (int)(Kp * err + Kd * diff);

    if (out > 25)  out = 25;
    if (out < -25) out = -25;

    last_err = err;
    return out;
}

int pid_angle2(int target, int yaw) {
    static int err_last = 0;

    int yaw0_360 = (yaw < 0) ? (yaw + 360) : yaw;
    int err = target - yaw0_360;

    if (err > 180)       err -= 360;
    else if (err < -180) err += 360;

    int diff = err - err_last;
    if (err_last == 0 && err != 0) {
        diff = err / 2;
    }

    int pid_a_out = (int)(KA2P * err + KA2D * diff);

    if (pid_a_out > 20)  pid_a_out = 20;
    if (pid_a_out < -20) pid_a_out = -20;

    err_last = err;
    return pid_a_out;
}

void Final_Speed(int pid_out, int base_speed) {
    int left_speed  = base_speed - pid_out;
    int right_speed = base_speed + pid_out;

    if (left_speed  < 5)  left_speed  = 5;
    if (left_speed  > 40) left_speed  = 40;
    if (right_speed < 5)  right_speed = 5;
    if (right_speed > 40) right_speed = 40;

    Set_Speed(left_speed, right_speed);
}
