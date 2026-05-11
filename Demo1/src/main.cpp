#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "mpu6050_dmp.h"
#include "inv_mpu.h"

TwoWire OLED_WIRE(PB11, PB10);
Adafruit_SSD1306 display(128, 32, &OLED_WIRE, -1);

float Yaw = 0;
uint8_t all_state = 0;
int step = 1;
int motorL_pwm = 0, motorR_pwm = 0;
int lost_cnt = 0;
float yaw_arc_start = 0;
char track_state = 'T';   // T=tracking, L=lost, X=arc_exit
uint16_t gray_threshold = 2400;
uint16_t gray_min_val = 0;

// ==== DMP Yaw ====
void readDmpYaw() {
    float p, r, y;
    if (MPU6050_DMP_Get_Data(&p, &r, &y) == 0) {
        Yaw = y;
    }
}

uint8_t readGrayBits();  // fwd decl

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

// ==== Keys (LOW=pressed, matches Keil) ====
uint8_t readKey() {
    if (digitalRead(PC9) == LOW) { delay(20); while(digitalRead(PC9)==LOW); delay(20); return 1; }
    if (digitalRead(PC8) == LOW) { delay(20); while(digitalRead(PC8)==LOW); delay(20); return 2; }
    return 0;
}

// ==== Motor driver (PA3/PA4=left dir, PA0/PA5=right dir, PA2/PA1=PWM) ====
void setMotor(int L, int R) {
    if (L >= 0) { digitalWrite(PA3,HIGH); digitalWrite(PA4,LOW); }
    else        { digitalWrite(PA3,LOW);  digitalWrite(PA4,HIGH); L=-L; }
    if (R >= 0) { digitalWrite(PA0,HIGH); digitalWrite(PA5,LOW); }
    else        { digitalWrite(PA0,LOW);  digitalWrite(PA5,HIGH); R=-R; }
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

// ==== PID: Angle PD (ka2p=0.5, ka2d=3.5, out [-8,8]) ====
int pidAngle(int tgt, int yaw) {
    static int last=0;
    int y=(yaw<0)?yaw+360:yaw;
    int e=tgt-y;
    if(e>180)e-=360; else if(e<-180)e+=360;
    int d=e-last; if(last==0 && e!=0) d=e/2;
    int o=(int)(0.5*e + 3.5*d);
    if(o>8)o=8; if(o<-8)o=-8;
    last=e; return o;
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

void setup() {
    pinMode(PB9,OUTPUT); digitalWrite(PB9,HIGH);
    pinMode(PB8,OUTPUT); digitalWrite(PB8,HIGH);
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
    uint16_t cal_max[8] = {0};
    for (int s = 0; s < 50; s++) {
        uint16_t vals[8]; readGray(vals);
        for (int i = 0; i < 8; i++) {
            if (vals[i] > cal_max[i]) cal_max[i] = vals[i];
        }
        delay(5);
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
    if (key == 2) all_state |= 0x10;

    switch (all_state) {
    case 0x11:
        delay(200);
        while (readGrayBits() == 0) {
            readDmpYaw();
            int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
            int p = pidAngle(0, yi);
            int L=12-p, R=12+p; if(L>20)L=20; if(L<-20)L=-20; if(R>20)R=20; if(R<-20)R=-20;
            setMotor(L, R);
        }
        setMotor(0, 0);
        all_state &= 0x0F;  // stop, back to selection
        digitalWrite(PB9,LOW); delay(100); digitalWrite(PB9,HIGH);
        break;

    case 0x12: {
        digitalWrite(PB9,HIGH); delay(100);
        while (step != 0) {
            switch (step) {
            case 1:
                while (readGrayBits() == 0) {
                    readDmpYaw();
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(0, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=2; break;
            case 2:
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > 160) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(8, 14);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErr(ir);
                        int p = pidTrack(fe, 0);
                        int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=3; break;
            case 3:
                while (readGrayBits() == 0) {
                    readDmpYaw();
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(174, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=4; break;
            case 4:
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > 160) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(8, 14);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErr(ir);
                        int p = pidTrack(fe, 0);
                        int L=10-p, R=10+p; if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=5; break;
            case 5: setMotor(0,0); all_state &= 0x0F; step = 0; break;
            }
        }
    } break;

    case 0x13: {
        digitalWrite(PB9,HIGH); delay(10);
        while (step != 0) {
            switch (step) {
            case 1: { delay(100);
                uint32_t t0 = millis();
                while (1) {
                    readDmpYaw();
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(-38, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                    if (millis() - t0 > 3000 && readGrayBits() != 0) {
                        delay(10);
                        if (readGrayBits() != 0) break;
                    }
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=2; break; }
            case 2: {
                uint32_t t0 = millis();
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > 40 && millis() - t0 > 3000) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(5, 15);
                        if (lost_cnt < 25) delay(5);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErr(ir);
                        int p = pidTrack(fe, 0);
                        int L=8-p, R=8+p; if(L<2)L=2;if(L>14)L=14;if(R<2)R=2;if(R>14)R=14; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=3; break; }
            case 3: {
                uint32_t t0 = millis();
                while (1) {
                    readDmpYaw();
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(-156, yi);
                    int L=14-p, R=14+p; if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20; setMotor(L,R);
                    if (millis() - t0 > 3000 && readGrayBits() != 0) {
                        delay(10);
                        if (readGrayBits() != 0) break;
                    }
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=4; break; }
            case 4: {
                uint32_t t0 = millis();
                yaw_arc_start = Yaw;
                lost_cnt = 0;
                while (1) {
                    uint8_t ir = readGrayBits();
                    if (ir == 0) {
                        lost_cnt++;
                        if (lost_cnt > 40 && millis() - t0 > 3000) { track_state = 'X'; break; }
                        track_state = 'L';
                        setMotor(15, 5);
                        if (lost_cnt < 25) delay(5);
                    } else {
                        lost_cnt = 0;
                        track_state = 'T';
                        float fe = getTrackErr(ir);
                        int p = pidTrack(fe, 0);
                        int L=8-p, R=8+p; if(L<2)L=2;if(L>14)L=14;if(R<2)R=2;if(R>14)R=14; setMotor(L,R);
                    }
                }
                setMotor(0,0);
                digitalWrite(PB9, HIGH); digitalWrite(PB8, HIGH);
                delay(500);
                digitalWrite(PB9, LOW);  digitalWrite(PB8, LOW);
                delay(50); pidTrack(0,0); step=5; break; }
            case 5: setMotor(0,0); all_state &= 0x0F; step = 0; break;
            }
        }
    } break;

    case 0x14: {
        digitalWrite(PB9,HIGH); delay(10);
        while (1) {
            switch (step) {
            case 1: delay(100);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(50,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=2;break;
            case 2: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=10-p,R=10+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=3;break;
            case 3: while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(129,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=4;break;
            case 4: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=8-p,R=8+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=5;break;
            case 5: delay(300);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(59,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=6;break;
            case 6: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=10-p,R=10+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=7;break;
            case 7: delay(300);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(137,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=8;break;
            case 8: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=8-p,R=8+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=9;break;
            case 9: delay(300);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(65,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=10;break;
            case 10: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=10-p,R=10+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=11;break;
            case 11: delay(300);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(145,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=12;break;
            case 12: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=8-p,R=8+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=13;break;
            case 13: delay(300);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(71,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=14;break;
            case 14: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=10-p,R=10+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=15;break;
            case 15: delay(300);
                while(readGrayBits()==0){readDmpYaw();int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(150,yi);int L=10-p,R=10+p;if(L>20)L=20;if(L<-20)L=-20;if(R>20)R=20;if(R<-20)R=-20;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=16;break;
            case 16: while(readGrayBits()!=0){float fe=getTrackErr(readGrayBits());int p=pidTrack(fe,0);int L=8-p,R=8+p;if(L<2)L=2;if(L>16)L=16;if(R<2)R=2;if(R>16)R=16;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=17;break;
            case 17: while(1){setMotor(0,0);} break;
            }
        }
    } break;
    }

}
