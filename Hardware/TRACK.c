#include "stm32f10x.h"                  
#include "TRACK.h" 
#include "Motor.h"
#include "stdio.h" 
#include "PWM.h"

extern uint8_t TRACK1;
extern uint8_t TRACK2;
extern uint8_t TRACK3;
extern uint8_t TRACK4;
extern uint8_t TRACK5;
extern uint8_t TRACK6;
extern uint8_t TRACK7;
extern uint8_t TRACK8;

void Track_Init(void)
{
	//A口的引脚配置
	GPIO_InitTypeDef GPIO_InitStructure;//GPIO结构体定义
	RCC_APB2PeriphClockCmd(TRACK_PORT_CLKA ,ENABLE);//打开端口时钟
	GPIO_InitStructure.GPIO_Pin = TRACK_INFRARED_PIN_1 | TRACK_INFRARED_PIN_2 | TRACK_INFRARED_PIN_3 | TRACK_INFRARED_PIN_5 | TRACK_INFRARED_PIN_7;//配置传感器读取引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//配置为输入上拉模式
	GPIO_Init(TRACK_PORTA,&GPIO_InitStructure);//初始化端口
	
	//B口的引脚配置
	RCC_APB2PeriphClockCmd(TRACK_PORT_CLKB ,ENABLE);      //打开端口时钟
	GPIO_InitStructure.GPIO_Pin = TRACK_INFRARED_PIN_4 | TRACK_INFRARED_PIN_6  |  TRACK_INFRARED_PIN_8 ;  //配置传感器读取引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//配置为输入上拉模式
	GPIO_Init(TRACK_PORTB,&GPIO_InitStructure);//初始化端口
}

 u8 Get_Infrared_State(void)
{
	TRACK1= GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_1 )<<7 ;
	TRACK2= GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_2 )<<6 ;
	TRACK3= GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_3 )<<5 ;
	TRACK4= GPIO_ReadInputDataBit(TRACK_PORTB, TRACK_INFRARED_PIN_4 )<<4 ;
	TRACK5= GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_5 )<<3 ;
	TRACK6= GPIO_ReadInputDataBit(TRACK_PORTB, TRACK_INFRARED_PIN_6 )<<2 ;
	TRACK7= GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_7 )<<1 ;
	TRACK8= GPIO_ReadInputDataBit(TRACK_PORTB, TRACK_INFRARED_PIN_8 )<<0 ;	

	//1扫到黑线，0没扫到
	u8 state = 0;
	
	state=(u8)(TRACK1|TRACK2|TRACK3|TRACK4|TRACK5|TRACK6|TRACK7|TRACK8);//拼接成八位数据，最高位为为传感器的的左1，最低为为传感器的右1
	return state;
	
	
}





