#include "stm32f10x.h"                  // Device header

/**************************************************************************
函数功能：TIM3定时中断初始化
入口参数：arr：自动重装值（ARR）  psc：时钟预分频数（PSC）
返回  值：无
**************************************************************************/
void TIM3_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 修正1：使能TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
    // 定时器时基参数配置
    TIM_TimeBaseInitStruct.TIM_Period = arr;     // 自动重装值ARR
    TIM_TimeBaseInitStruct.TIM_Prescaler = psc;  // 预分频系数PSC
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0; // 时钟分割
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    // 修正2：初始化TIM3
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
    // 修正3：使能TIM3的更新（溢出）中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
    // 中断优先级配置
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;   // TIM3对应的中断通道
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;   // 使能该中断通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级1（需配合中断分组）
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;    // 子优先级3
    NVIC_Init(&NVIC_InitStruct);

   
    TIM_Cmd(TIM3, ENABLE);	  // 使能定时器3
}

