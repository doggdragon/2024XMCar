#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "mpu6050_dmp.h"
#include "inv_mpu.h"

TwoWire OLED_WIRE(PB11, PB10);
Adafruit_SSD1306 display(128, 32, &OLED_WIRE, -1);

float Yaw = 0;
const float YAW_ZERO_BIAS = -4.0f;
uint8_t all_state = 0;
int step = 1;
int motorL_pwm = 0, motorR_pwm = 0;
float angle_pid_last = 0.0f;
int lost_cnt = 0;
float yaw_arc_start = 0;
char track_state = 'T';   // T=tracking, L=lost, X=arc_exit
uint16_t gray_threshold = 2400;
uint16_t gray_min_val = 0;
uint16_t gray_white_base[8] = {0};
uint8_t point_b_active_count = 0;
const uint16_t POINT_B_DELTA_THRESHOLD = 250;
const float MODE1_TARGET_YAW = 1.0f;
const int MODE1_BASE_SPEED = 12;
const int MODE1_LEFT_SPEED_TRIM = 1;
const int MODE1_RIGHT_SPEED_TRIM = 0;
const float MODE2_CD_TARGET_YAW = 174.5;
const float MODE2_ARC1_TRACK_TARGET = 1.7f;
const float MODE2_ARC2_TRACK_TARGET = 0.0f;
const float MODE3_AC_TARGET_YAW = -38.0f;
const float MODE3_BD_TARGET_YAW = -146.2f;
const uint32_t MODE3_STRAIGHT_MIN_TIME = 3000;
const uint32_t MODE3_ARC_MIN_TIME = 3000;
const int MODE3_ARC_LOST_LIMIT = 40;
const float MODE3_ARC1_TRACK_TARGET = 0.0f;
const float MODE3_ARC2_TRACK_TARGET = 0.0f;
const uint8_t MODE4_LAP_COUNT = 4;
const float MODE4_AC_TARGET_YAW[MODE4_LAP_COUNT] = {-38.0f, -34.0f, -34.0f, -34.0f};
const float MODE4_BD_TARGET_YAW[MODE4_LAP_COUNT] = {-146.2f, -146.2f, -146.2f, -146.2f};
const float MODE4_ARC1_TRACK_TARGET[MODE4_LAP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
const float MODE4_ARC2_TRACK_TARGET[MODE4_LAP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
const uint8_t LIGHT_ON = HIGH;
const uint8_t LIGHT_OFF = LOW;
const uint8_t BUZZER_ON = LOW;
const uint8_t BUZZER_OFF = HIGH;

// ==== DMP Yaw ====
float normalizeYaw(float yaw) {
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw <= -180.0f) yaw += 360.0f;
    return yaw;
}

float yawTo360(float yaw) {
    return (yaw < 0.0f) ? (yaw + 360.0f) : yaw;
}

void readDmpYaw() {
    float p, r, y;
    if (MPU6050_DMP_Get_Data(&p, &r, &y) == 0) {
        Yaw = normalizeYaw(y - YAW_ZERO_BIAS);
    }
}

uint8_t readGrayBits();  // fwd decl
void setMotor(int L, int R);
int pidAngle(float tgt, float yaw);

// ==== OLED tick ====
void oledTick() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 200) return;
    last = now;
    display.clearDisplay();
    display.setCursor(0,0); display.print(F("Y:")); display.print((int)Yaw);
    display.print(F(" L:")); display.print(motorL_pwm); display.print(F(" R:")); display.print(motorR_pwm);
    display.setCursor(0,10);
    display.print(F("M:")); display.print(all_state,HEX);
    display.print(F(" S:")); display.print(step);
    display.print(F(" "));
    for (int i = 1; i <= 4; i++) {
        if (i == step) { display.print(F("[")); display.print(i); display.print(F("]")); }
        else { display.print(F(" ")); display.print(i); }
    }
    display.setCursor(0,20);
    display.print(F("lost:")); display.print(lost_cnt);
    display.print(F(" G:")); display.print(point_b_active_count);
    display.print(F(" ")); display.print(track_state);
    display.display();
}

// ==== Grayscale sensor (8ch mux: AD0/1/2 select channel, OUT=analog) ====
#define GRAY_AD0 PB13
#define GRAY_AD1 PB14
#define GRAY_AD2 PB15
#define GRAY_OUT PA6

void readGray(uint16_t *vals) {
    for (int ch = 0; ch < 8; ch++) {
        digitalWrite(GRAY_AD0, ch & 1);
        digitalWrite(GRAY_AD1, (ch >> 1) & 1);
        digitalWrite(GRAY_AD2, (ch >> 2) & 1);
        delayMicroseconds(20);
        vals[ch] = analogRead(GRAY_OUT);
    }
}

uint8_t readGrayBits() {
    uint16_t vals[8];
    readGray(vals);
    uint8_t bits = 0;
    gray_min_val = 4095;
    for (int i = 0; i < 8; i++) {
        if (vals[i] < gray_threshold) bits |= (1 << (7 - i));
        if (vals[i] < gray_min_val) gray_min_val = vals[i];
    }
    return bits;
}

uint8_t countGrayActive(uint8_t bits) {
    uint8_t count = 0;
    for (int i = 0; i < 8; i++) {
        if (bits & (1 << i)) count++;
    }
    return count;
}

uint8_t readCalibratedGrayBits() {
    uint16_t vals[8];
    readGray(vals);
    uint8_t bits = 0;
    for (int i = 0; i < 8; i++) {
        uint16_t diff = (vals[i] > gray_white_base[i])
            ? (vals[i] - gray_white_base[i])
            : (gray_white_base[i] - vals[i]);
        if (diff >= POINT_B_DELTA_THRESHOLD) bits |= (1 << (7 - i));
    }
    point_b_active_count = countGrayActive(bits);
    return bits;
}

bool isPointBDetected() {
    return countGrayActive(readCalibratedGrayBits()) >= 2;
}

bool isPointDetected() {
    return isPointBDetected();
}

void soundLightHint() {
    digitalWrite(PB9, LIGHT_ON);
    digitalWrite(PB8, BUZZER_ON);
    delay(120);
    digitalWrite(PB9, LIGHT_OFF);
    digitalWrite(PB8, BUZZER_OFF);
    delay(80);
}

void setSoundLightOff() {
    digitalWrite(PB9, LIGHT_OFF);
    digitalWrite(PB8, BUZZER_OFF);
}

void alignYawToTarget(float target) {
    angle_pid_last = 0.0f;
    uint8_t stable_count = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 1800) {
        readDmpYaw();
        float yi = yawTo360(Yaw);
        int p = pidAngle(target, yi);
        if (p > -2 && p < 2) {
            stable_count++;
            setMotor(0, 0);
            if (stable_count >= 8) break;
            delay(15);
        } else {
            stable_count = 0;
            int turn = p;
            if (turn > 0 && turn < 4) turn = 4;
            if (turn < 0 && turn > -4) turn = -4;
            setMotor(-turn, turn);
        }
    }
    setMotor(0, 0);
}

// ==== Keys (LOW=pressed, matches Keil) ====
uint8_t readKey() {
    if (digitalRead(PC9) == LOW) { delay(20); while(digitalRead(PC9)==LOW); delay(20); return 1; }
    if (digitalRead(PC8) == LOW) { delay(20); while(digitalRead(PC8)==LOW); delay(20); return 2; }
    return 0;
}

// ==== Motor driver (PA3/PA4=left dir, PA0/PA5=right dir, PA2/PA1=PWM) ====
void setMotor(int L, int R) {
    if (L >= 0) { digitalWrite(PA3,LOW);  digitalWrite(PA4,HIGH); }
    else        { digitalWrite(PA3,HIGH); digitalWrite(PA4,LOW);  L=-L; }
    if (R >= 0) { digitalWrite(PA0,LOW);  digitalWrite(PA5,HIGH); }
    else        { digitalWrite(PA0,HIGH); digitalWrite(PA5,LOW);  R=-R; }
    if (L>20) L=20; if (R>20) R=20;
    motorL_pwm = L; motorR_pwm = R;
    analogWrite(PA2, L*255/50);  // PA2=left PWM
    analogWrite(PA1, R*255/50);  // PA1=right PWM
    oledTick();
}

// ==== PID: Track PD (Kp=2.5, Kd=3.5, out [-10,10]) ====
float trackErr;
int pidTrack(float err, int tgt) {
    static int last=0;
    int e=(int)err;
    int d=e-last; if(last==0 && e!=0) d=e/2;
    int o=(int)(2.5*e + 3.5*d);
    if(o>10)o=10; if(o<-10)o=-10;
    last=e; return o;
}

// ==== PID: Mode 2 centroid track PD ====
int pidTrackCentroid(float err, float tgt) {
    static float last = 0.0f;
    float e = err - tgt;
    float d = e - last;
    if (last == 0.0f && e != 0.0f) d = e * 0.5f;
    int o = (int)(2.5f * e + 3.5f * d);
    if (o > 10) o = 10; if (o < -10) o = -10;
    last = e; return o;
}

// ==== PID: Angle PD (ka2p=0.5, ka2d=3.5, out [-8,8]) ====
int pidAngle(float tgt, float yaw) {
    float y = yawTo360(yaw);
    float e = tgt - y;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    float d=e-angle_pid_last; if(angle_pid_last==0.0f && e!=0.0f) d=e*0.5f;
    int o=(int)(0.5f*e + 3.5f*d);
    if(o>8)o=8; if(o<-8)o=-8;
    angle_pid_last=e; return o;
}

// ==== IR error lookup (matches Cnotorl.c) ====
float getTrackErr(uint8_t s) {
    switch(s) {
        case 0x00: return 0;
        case 0x10: case 0x08: return 1;
        case 0x18: case 0x3C: case 0x7E: return 0;
        case 0x30: case 0x20: return 2;
        case 0x40: case 0x60: return 4;
        case 0x80: case 0xC0: case 0xE0: case 0xA0:
        case 0xFE: case 0xFC: case 0xF8: case 0xF0: return 6;
        case 0x7C: case 0x78: return 4;
        case 0x38: case 0x0C: case 0x1C: return 2;
        case 0x04: return -2;
        case 0x0E: case 0x1E: case 0x3E: return -4;
        case 0x1F: case 0x3F: case 0x7D: return -6;
        case 0x02: case 0x06: return -4;
        case 0x01: case 0x03: case 0x07: case 0x0F: return -6;
        default: return trackErr;
    }
}

float getTrackErrCentroid(uint8_t s) {
    static const int8_t pos[8] = {6, 4, 2, 1, -1, -2, -4, -6};  // X1..X8
    static const uint8_t weight[8] = {10, 10, 10, 20, 20, 10, 10, 10};
    int32_t weighted_sum = 0;
    int32_t weight_sum = 0;

    for (int i = 0; i < 8; i++) {
        if (s & (1 << (7 - i))) {
            weighted_sum += (int32_t)pos[i] * weight[i];
            weight_sum += weight[i];
        }
    }
    if (weight_sum == 0) return trackErr;

    trackErr = (float)weighted_sum / weight_sum;
    return trackErr;
}

void resetTrackControl() {
    pidTrack(0, 0);
    pidTrackCentroid(0, 0);
}

void stopAtPointWithHint() {
    setMotor(0, 0);
    soundLightHint();
    delay(50);
    resetTrackControl();
}

void runMode3StraightToPoint(float target_yaw) {
    angle_pid_last = 0.0f;
    uint32_t t0 = millis();
    while (1) {
        readDmpYaw();
        float yi = yawTo360(Yaw);
        int p = pidAngle(target_yaw, yi);
        int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
        if (millis() - t0 > MODE3_STRAIGHT_MIN_TIME && isPointDetected()) {
            delay(10);
            if (isPointDetected()) break;
        }
    }
    stopAtPointWithHint();
}

void runMode3ArcToPoint(float track_target, int lost_l, int lost_r) {
    uint32_t t0 = millis();
    yaw_arc_start = Yaw;
    lost_cnt = 0;
    while (1) {
        uint8_t ir = readCalibratedGrayBits();
        if (ir == 0) {
            lost_cnt++;
            if (lost_cnt > MODE3_ARC_LOST_LIMIT && millis() - t0 > MODE3_ARC_MIN_TIME) { track_state = 'X'; break; }
            track_state = 'L';
            setMotor(lost_l, lost_r);
            if (lost_cnt < 25) delay(5);
        } else {
            lost_cnt = 0;
            track_state = 'T';
            float fe = getTrackErrCentroid(ir);
            int p = pidTrackCentroid(fe, track_target);
            int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16; setMotor(L,R);
        }
    }
    stopAtPointWithHint();
}

void runRequirement3Lap(bool align_start, float ac_yaw, float bd_yaw, float arc1_target, float arc2_target) {
    if (align_start) alignYawToTarget(ac_yaw);
    step = 1;
    delay(100);
    runMode3StraightToPoint(ac_yaw);

    step = 2;
    runMode3ArcToPoint(arc1_target, 5, 15);

    step = 3;
    alignYawToTarget(bd_yaw);
    runMode3StraightToPoint(bd_yaw);

    step = 4;
    runMode3ArcToPoint(arc2_target, 15, 5);
}

void setup() {
    pinMode(PB9,OUTPUT); digitalWrite(PB9,LIGHT_OFF);
    pinMode(PB8,OUTPUT); digitalWrite(PB8,BUZZER_OFF);
    OLED_WIRE.begin(); OLED_WIRE.setClock(400000);
    display.begin(SSD1306_SWITCHCAPVCC,0x3C);
    display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

    pinMode(PC9,INPUT_PULLUP); pinMode(PC8,INPUT_PULLUP);
    pinMode(GRAY_AD0,OUTPUT); pinMode(GRAY_AD1,OUTPUT); pinMode(GRAY_AD2,OUTPUT);
    pinMode(GRAY_OUT,INPUT_ANALOG);
    analogReadResolution(12);
    pinMode(PA3,OUTPUT); pinMode(PA4,OUTPUT); pinMode(PA0,OUTPUT); pinMode(PA5,OUTPUT);
    digitalWrite(PA3,LOW); digitalWrite(PA4,LOW); digitalWrite(PA0,LOW); digitalWrite(PA5,LOW);
    pinMode(PA2,OUTPUT); pinMode(PA1,OUTPUT); analogWrite(PA2,0); analogWrite(PA1,0);

    // Grayscale calibration (sample white surface)
    uint32_t cal_sum[8] = {0};
    uint16_t cal_max[8] = {0};
    for (int s = 0; s < 50; s++) {
        uint16_t vals[8]; readGray(vals);
        for (int i = 0; i < 8; i++) {
            cal_sum[i] += vals[i];
            if (vals[i] > cal_max[i]) cal_max[i] = vals[i];
        }
        delay(5);
    }
    for (int i = 0; i < 8; i++) {
        gray_white_base[i] = cal_sum[i] / 50;
    }
    uint16_t cal_min = cal_max[0];
    for (int i = 1; i < 8; i++) {
        if (cal_max[i] < cal_min) cal_min = cal_max[i];
    }
    gray_threshold = cal_min * 0.7f;

    Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(100000);

    u8 dmp_r = MPU6050_DMP_Init();
}

void loop() {
    readDmpYaw();
    oledTick();

    uint8_t ir = readGrayBits();
    uint8_t key = readKey();
    if (key == 1) {
        uint8_t s = all_state & 0x0F;
        if (s==0) all_state=0x01; else if (s==1) all_state=0x02;
        else if (s==2) all_state=0x03; else if (s==3) all_state=0x04;
        else if (s==4) all_state=0x01;
    }
    if (key == 2) {
        step = 1;
        lost_cnt = 0;
        track_state = 'T';
        point_b_active_count = 0;
        all_state |= 0x10;
    }

    switch (all_state) {
    case 0x11:
        delay(200);
        while (!isPointBDetected()) {
            readDmpYaw();
            float yi = yawTo360(Yaw);
            int p = pidAngle(MODE1_TARGET_YAW, yi);
            int L=MODE1_BASE_SPEED+MODE1_LEFT_SPEED_TRIM-p, R=MODE1_BASE_SPEED+MODE1_RIGHT_SPEED_TRIM+p; if(L>20)L=20; if(L<-20)L=-20; if(R>20)R=20; if(R<-20)R=-20;
            setMotor(L, R);
        }
        setMotor(0, 0);
        all_state &= 0x0F;  // stop, back to selection
        soundLightHint();
        break;

    case 0x12: {
        setSoundLightOff(); delay(100);
        while (step != 0) {
            switch (step) {
            case 1:
                while (!isPointBDetected()) {
                    readDmpYaw();
                    float yi = yawTo360(Yaw);
                    int p = pidAngle(0.0f, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                }
                setMotor(0,0);
                soundLightHint();
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=2; break;
            case 2:
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readCalibratedGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > 160) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(8, 14);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErrCentroid(ir);
                        int p = pidTrackCentroid(fe, MODE2_ARC1_TRACK_TARGET);
                        int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                soundLightHint();
                alignYawToTarget(MODE2_CD_TARGET_YAW);
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=3; break;
            case 3:
                while (!isPointBDetected()) {
                    readDmpYaw();
                    float yi = yawTo360(Yaw);
                    int p = pidAngle(MODE2_CD_TARGET_YAW, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                }
                setMotor(0,0);
                soundLightHint();
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=4; break;
            case 4:
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readCalibratedGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > 160) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(8, 14);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErrCentroid(ir);
                        int p = pidTrackCentroid(fe, MODE2_ARC2_TRACK_TARGET);
                        int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                soundLightHint();
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=5; break;
            case 5: setMotor(0,0); all_state &= 0x0F; step = 0; break;
            }
        }
    } break;

    case 0x13: {
        setSoundLightOff(); delay(10);
        while (step != 0) {
            switch (step) {
            case 1: {  // A -> C, straight by DMP
                delay(100);
                uint32_t t0 = millis();
                while (1) {
                    readDmpYaw();
                    float yi = yawTo360(Yaw);
                    int p = pidAngle(MODE3_AC_TARGET_YAW, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                    if (millis() - t0 > MODE3_STRAIGHT_MIN_TIME && isPointDetected()) {
                        delay(10);
                        if (isPointDetected()) break;
                    }
                }
                setMotor(0,0);
                soundLightHint();
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=2; break; }
            case 2: {  // C -> B, half-arc line tracking
                uint32_t t0 = millis();
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readCalibratedGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > MODE3_ARC_LOST_LIMIT && millis() - t0 > MODE3_ARC_MIN_TIME) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(5, 15);
                        if (lost_cnt < 25) delay(5);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErrCentroid(ir);
                        int p = pidTrackCentroid(fe, MODE3_ARC1_TRACK_TARGET);
                        int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16                                         ; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                soundLightHint();
                alignYawToTarget(MODE3_BD_TARGET_YAW);
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=3; break; }
            case 3: {  // B -> D, straight by DMP
                uint32_t t0 = millis();
                while (1) {
                    readDmpYaw();
                    float yi = yawTo360(Yaw);
                    int p = pidAngle(MODE3_BD_TARGET_YAW, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                    if (millis() - t0 > MODE3_STRAIGHT_MIN_TIME && isPointDetected()) {
                        delay(10);
                        if (isPointDetected()) break;
                    }
                }
                setMotor(0,0);
                soundLightHint();
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=4; break; }
            case 4: {  // D -> A, half-arc line tracking
                uint32_t t0 = millis();
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readCalibratedGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > MODE3_ARC_LOST_LIMIT && millis() - t0 > MODE3_ARC_MIN_TIME) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(15, 5);
                        if (lost_cnt < 25) delay(5);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErrCentroid(ir);
                        int p = pidTrackCentroid(fe, MODE3_ARC2_TRACK_TARGET);
                        int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16                                         ; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                soundLightHint();
                delay(50); pidTrack(0,0); pidTrackCentroid(0,0); step=5; break; }
            case 5: setMotor(0,0); all_state &= 0x0F; step = 0; break;
            }
        }
    } break;

    case 0x14: {
        setSoundLightOff(); delay(10);
        for (uint8_t lap = 0; lap < MODE4_LAP_COUNT; lap++) {
            runRequirement3Lap(
                true,
                MODE4_AC_TARGET_YAW[lap],
                MODE4_BD_TARGET_YAW[lap],
                MODE4_ARC1_TRACK_TARGET[lap],
                MODE4_ARC2_TRACK_TARGET[lap]
            );
        }
        setMotor(0,0);
        all_state &= 0x0F;
        step = 0;
    } break;
    }

}
