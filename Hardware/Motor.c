#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_5 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    PWM_Init();
}


int myabs(int a)//取绝对值函数
{
	int temp;
	if(a>=0)  temp=a;
	else      temp=-a;
	return temp;

}

void Limit(int *motor_left,int *motor_right)
{
	if (*motor_left>PWM_MAX)  *motor_left=PWM_MAX;
	if (*motor_left<PWM_MIN)  *motor_left=PWM_MIN;
	
    if (*motor_right>PWM_MAX)  *motor_right=PWM_MAX;
	if (*motor_right<PWM_MIN)  *motor_right=PWM_MIN;

}
	
void Set_Speed(int motor_l ,int motor_r)
{
   if (motor_l>=0) GPIO_SetBits(GPIOA, GPIO_Pin_3),GPIO_ResetBits(GPIOA, GPIO_Pin_4), TIM_SetCompare3(TIM2, motor_l);
    else           GPIO_SetBits(GPIOA, GPIO_Pin_4),GPIO_ResetBits(GPIOA, GPIO_Pin_3), TIM_SetCompare3(TIM2, -motor_l);
   
   	
	if (motor_r>=0) GPIO_SetBits(GPIOA, GPIO_Pin_0),GPIO_ResetBits(GPIOA, GPIO_Pin_5),TIM_SetCompare2(TIM2, motor_r);	
    else            GPIO_SetBits(GPIOA, GPIO_Pin_5),GPIO_ResetBits(GPIOA, GPIO_Pin_0),TIM_SetCompare2(TIM2, -motor_r);	   
}
