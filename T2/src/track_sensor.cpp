#include "track_sensor.h"
#include "pins.h"

static const uint8_t track_pins[8] = {
    PIN_TRACK1, PIN_TRACK2, PIN_TRACK3, PIN_TRACK4,
    PIN_TRACK5, PIN_TRACK6, PIN_TRACK7, PIN_TRACK8
};

void Track_Init() {
    for (int i = 0; i < 8; i++) {
        pinMode(track_pins[i], INPUT_PULLUP);
    }
}

uint8_t Get_Infrared_State() {
    uint8_t state = 0;
    // TRACK1 = MSB (bit7), TRACK8 = LSB (bit0)
    // Active LOW: line=0, ground=1 → invert so line=1
    for (int i = 0; i < 8; i++) {
        if (digitalRead(track_pins[i])) {
            state |= (1 << (7 - i));
        }
    }
    return state;
}
