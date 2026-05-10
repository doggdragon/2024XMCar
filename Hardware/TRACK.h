#include "stm32f10x.h"                  

#ifndef __TRACK_H
#define __TRACK_H

void Track_Adjust(void);
void Track_Init(void);

#define TRACK_PORTA 							GPIOA
#define TRACK_PORTB 							GPIOB

#define TRACK_PORT_CLKA					RCC_APB2Periph_GPIOA
#define TRACK_PORT_CLKB					RCC_APB2Periph_GPIOB

 u8 Get_Infrared_State(void);

#define TRACK_INFRARED_PIN_1		GPIO_Pin_15       //  A
#define TRACK_INFRARED_PIN_2		GPIO_Pin_8        //  A
#define TRACK_INFRARED_PIN_3		GPIO_Pin_11       //  A
#define TRACK_INFRARED_PIN_4		GPIO_Pin_15       //  B
#define TRACK_INFRARED_PIN_5		GPIO_Pin_7       //  A
#define TRACK_INFRARED_PIN_6		GPIO_Pin_14       //  B
#define TRACK_INFRARED_PIN_7		GPIO_Pin_6       //  A
#define TRACK_INFRARED_PIN_8		GPIO_Pin_13       //  B

#endif


