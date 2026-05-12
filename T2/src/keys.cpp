#include "keys.h"
#include "pins.h"

void Key_Init() {
    pinMode(PIN_KEY1, INPUT_PULLUP);
    pinMode(PIN_KEY2, INPUT_PULLUP);
}

uint8_t Key_GetNum() {
    uint8_t KeyNum = 0;
    if (digitalRead(PIN_KEY1) == LOW) {
        delay(20);
        while (digitalRead(PIN_KEY1) == LOW);
        delay(20);
        KeyNum = 1;
    }
    if (digitalRead(PIN_KEY2) == LOW) {
        delay(20);
        while (digitalRead(PIN_KEY2) == LOW);
        delay(20);
        KeyNum = 2;
    }
    return KeyNum;
}
