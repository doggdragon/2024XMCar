#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TwoWire OLED_WIRE(PB11, PB10);
Adafruit_SSD1306 display(128, 32, &OLED_WIRE, -1);

float Yaw = 0;
unsigned long last_us = 0;
uint8_t all_state = 0;
int step = 1;

// ==== MPU6050 raw gyro Z ====
int16_t readGyroZ() {
    Wire.beginTransmission(0x68);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2);
    return (Wire.read() << 8) | Wire.read();
}

// ==== IR sensors (matches Keil: bit=1 means line detected) ====
uint8_t readIR() {
    uint8_t ir = 0;
    if (digitalRead(PA15)) ir |= 0x80;
    if (digitalRead(PA8))  ir |= 0x40;
    if (digitalRead(PA11)) ir |= 0x20;
    if (digitalRead(PB15)) ir |= 0x10;
    if (digitalRead(PA7))  ir |= 0x08;
    if (digitalRead(PB14)) ir |= 0x04;
    if (digitalRead(PA6))  ir |= 0x02;
    if (digitalRead(PB13)) ir |= 0x01;
    return ir;
}

// ==== Keys (LOW=pressed, matches Keil) ====
uint8_t readKey() {
    if (digitalRead(PB4) == LOW) { delay(20); while(digitalRead(PB4)==LOW); delay(20); return 1; }
    if (digitalRead(PB3) == LOW) { delay(20); while(digitalRead(PB3)==LOW); delay(20); return 2; }
    return 0;
}

// ==== Motor driver (PA3/PA4=left, PA0/PA5=right, PA2/PA1=PWM) ====
void setMotor(int L, int R) {
    if (L >= 0) { digitalWrite(PA3,HIGH); digitalWrite(PA4,LOW); }
    else        { digitalWrite(PA3,LOW);  digitalWrite(PA4,HIGH); L=-L; }
    if (R >= 0) { digitalWrite(PA0,HIGH); digitalWrite(PA5,LOW); }
    else        { digitalWrite(PA0,LOW);  digitalWrite(PA5,HIGH); R=-R; }
    if (L>50) L=50; if (R>50) R=50;
    analogWrite(PA2, L*255/50);  // PA2=left PWM
    analogWrite(PA1, R*255/50);  // PA1=right PWM
}

// ==== PID: Track PD (Kp=2.5, Kd=3.5, out [-25,25]) ====
float trackErr;
int pidTrack(float err, int tgt) {
    static int last=0;
    int e=(int)err;
    int d=e-last; if(last==0 && e!=0) d=e/2;
    int o=(int)(2.5*e + 3.5*d);
    if(o>25)o=25; if(o<-25)o=-25;
    last=e; return o;
}

// ==== PID: Angle PD (ka2p=0.5, ka2d=3.5, out [-20,20]) ====
int pidAngle(int tgt, int yaw) {
    static int last=0;
    int y=(yaw<0)?yaw+360:yaw;
    int e=tgt-y;
    if(e>180)e-=360; else if(e<-180)e+=360;
    int d=e-last; if(last==0 && e!=0) d=e/2;
    int o=(int)(0.5*e + 3.5*d);
    if(o>20)o=20; if(o<-20)o=-20;
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

    RCC->APB2ENR|=0x00000001; AFIO->MAPR=(AFIO->MAPR&~0x07000000)|0x02000000;
    pinMode(PB4,INPUT_PULLUP); pinMode(PB3,INPUT_PULLUP);
    pinMode(PA15,INPUT_PULLUP); pinMode(PA8,INPUT_PULLUP); pinMode(PA11,INPUT_PULLUP); pinMode(PB15,INPUT_PULLUP);
    pinMode(PA7,INPUT_PULLUP); pinMode(PB14,INPUT_PULLUP); pinMode(PA6,INPUT_PULLUP); pinMode(PB13,INPUT_PULLUP);
    pinMode(PA3,OUTPUT); pinMode(PA4,OUTPUT); pinMode(PA0,OUTPUT); pinMode(PA5,OUTPUT);
    digitalWrite(PA3,LOW); digitalWrite(PA4,LOW); digitalWrite(PA0,LOW); digitalWrite(PA5,LOW);
    pinMode(PA2,OUTPUT); pinMode(PA1,OUTPUT); analogWrite(PA2,0); analogWrite(PA1,0);

    Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(400000);
    Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    delay(100);
    Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x18); Wire.endTransmission();

    display.clearDisplay();
    display.setCursor(0,0); display.println(F("READY"));
    display.setCursor(0,10); display.print(F("M:0x")); display.println(all_state,HEX);
    display.setCursor(0,20); display.println(F("Key1=mode Key2=run"));
    display.display();
    last_us = micros();
}

void loop() {
    // Update Yaw from gyro Z
    unsigned long now = micros();
    float dt = (now - last_us) / 1000000.0f;
    last_us = now;
    Yaw += (readGyroZ() / 16.4f) * dt;
    if (Yaw > 180) Yaw -= 360; else if (Yaw < -180) Yaw += 360;

    uint8_t ir = readIR();
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
        while (readIR() == 0) {
            int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
            int p = pidAngle(0, yi);
            int L=30-p, R=30+p; if(L>50)L=50; if(L<-50)L=-50; if(R>50)R=50; if(R<-50)R=-50;
            setMotor(L, R);
        }
        setMotor(0, 0);
        digitalWrite(PB9,LOW); delay(100); digitalWrite(PB9,HIGH);
        break;

    case 0x12: {
        digitalWrite(PB9,HIGH); delay(100);
        while (1) {
            switch (step) {
            case 1:
                while (readIR() == 0) {
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(0, yi);
                    int L=25-p, R=25+p; if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50; setMotor(L,R);
                }
                setMotor(0,0); delay(50); pidTrack(0,0); step=2; break;
            case 2:
                while (readIR() != 0) {
                    float fe = getTrackErr(readIR());
                    int p = pidTrack(fe, 0);
                    int L=25-p, R=25+p; if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40; setMotor(L,R);
                }
                if (readIR() == 0) { setMotor(0,0); delay(200); step=3; } else step=2; break;
            case 3:
                while (readIR() == 0) {
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(168, yi);
                    int L=30-p, R=30+p; if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50; setMotor(L,R);
                }
                setMotor(0,0); delay(50); pidTrack(0,0); step=4; break;
            case 4:
                while (readIR() != 0) {
                    float fe = getTrackErr(readIR());
                    int p = pidTrack(fe, 0);
                    int L=25-p, R=25+p; if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40; setMotor(L,R);
                }
                if (readIR()==0) step=5; else step=4; break;
            case 5: while(1){setMotor(0,0);} break;
            }
        }
    } break;

    case 0x13: {
        digitalWrite(PB9,HIGH); delay(10);
        while (1) {
            switch (step) {
            case 1: delay(100);
                while (readIR() == 0) {
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(50, yi);
                    int L=30-p, R=30+p; if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50; setMotor(L,R);
                }
                setMotor(0,0); delay(50); pidTrack(0,0); step=2; break;
            case 2:
                while (readIR() != 0) {
                    float fe = getTrackErr(readIR()); int p = pidTrack(fe, 0);
                    int L=25-p, R=25+p; if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40; setMotor(L,R);
                }
                setMotor(0,0); delay(50); pidTrack(0,0); step=3; break;
            case 3:
                while (readIR() == 0) {
                    int yi = (int)(((Yaw<0)?Yaw+360:Yaw) + 0.5f);
                    int p = pidAngle(130, yi);
                    int L=30-p, R=30+p; if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50; setMotor(L,R);
                }
                setMotor(0,0); delay(50); pidTrack(0,0); step=4; break;
            case 4:
                while (readIR() != 0) {
                    float fe = getTrackErr(readIR()); int p = pidTrack(fe, 0);
                    int L=22-p, R=22+p; if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40; setMotor(L,R);
                }
                setMotor(0,0); delay(50); pidTrack(0,0); step=5; break;
            case 5: while(1){setMotor(0,0);} break;
            }
        }
    } break;

    case 0x14: {
        digitalWrite(PB9,HIGH); delay(10);
        while (1) {
            switch (step) {
            case 1: delay(100);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(50,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=2;break;
            case 2: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=25-p,R=25+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=3;break;
            case 3: while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(129,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=4;break;
            case 4: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=22-p,R=22+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=5;break;
            case 5: delay(300);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(59,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=6;break;
            case 6: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=25-p,R=25+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=7;break;
            case 7: delay(300);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(137,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=8;break;
            case 8: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=22-p,R=22+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=9;break;
            case 9: delay(300);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(65,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=10;break;
            case 10: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=25-p,R=25+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=11;break;
            case 11: delay(300);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(145,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=12;break;
            case 12: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=22-p,R=22+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=13;break;
            case 13: delay(300);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(71,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=14;break;
            case 14: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=25-p,R=25+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=15;break;
            case 15: delay(300);
                while(readIR()==0){int yi=(int)(((Yaw<0)?Yaw+360:Yaw)+0.5f);int p=pidAngle(150,yi);int L=25-p,R=25+p;if(L>50)L=50;if(L<-50)L=-50;if(R>50)R=50;if(R<-50)R=-50;setMotor(L,R);}
                setMotor(0,0);delay(100);pidTrack(0,0);step=16;break;
            case 16: while(readIR()!=0){float fe=getTrackErr(readIR());int p=pidTrack(fe,0);int L=22-p,R=22+p;if(L<5)L=5;if(L>40)L=40;if(R<5)R=5;if(R>40)R=40;setMotor(L,R);}
                setMotor(0,0);delay(50);pidTrack(0,0);step=17;break;
            case 17: while(1){setMotor(0,0);} break;
            }
        }
    } break;
    }

    // OLED update
    display.clearDisplay();
    display.setCursor(0,0); display.print(F("Y:")); display.print((int)Yaw);
    display.setCursor(0,10); display.print(F("M:")); display.print(all_state,HEX);
    display.print(F(" S:")); display.print(step);
    display.setCursor(0,20); display.print(F("I:0x")); display.print(ir,HEX);
    display.display();
}
