#include "stm32f10x.h"              
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "TRACK.h"
#include "Cnotorl.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "buzzer.h"
#include "LED.h"
#include "uart.h"


uint8_t TRACK1;
uint8_t TRACK2; 
uint8_t TRACK3;
uint8_t TRACK4;
uint8_t TRACK5;
uint8_t TRACK6;
uint8_t TRACK7;
uint8_t TRACK8;


int pid_out;
int find_err;
int pid_angle2(int target,int yaw);

float target_angle=0;
float Pitch,Roll,Yaw;								//俯仰角默认跟中值一样，翻滚角，偏航角
int16_t ax,ay,az,gx,gy,gz;							//加速度，陀螺仪角速度

u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
int left=0,right=0  ,aleft=0,aright=0;
float Yaw0_360=0;
int ack=0;
uint8_t all_state=0;
int step=1;	
int count=0;

int main(void)
{	
	Track_Init();
	LED_Init();
	buzzer_Init();
	Motor_Init();
	Key_Init();
	
	OLED_Init();
	MPU6050_Init();
	MPU6050_DMP_Init();	
	
	while (1)
	{	
		buzzer_off();
		LED1_ON ();
		while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
		ack=1;
		Delay_ms (10);	
		OLED_ShowSignedNum(1,1,Yaw,5);
		OLED_ShowHexNum(2,1,all_state,4);
		OLED_ShowSignedNum(3,1,step,3);
		
		OLED_ShowBinNum(4,1,Get_Infrared_State(),8);		
		switch (Key_GetNum())
{
    case 1:
        // 提取低4位状态（第一问至第四问的状态）
        if ((all_state & 0x000f) == 0x0000)
            all_state = 0x0001;  // 从初始态→第一问
        else if ((all_state & 0x000f) == 0x0001)
            all_state = 0x0002;  // 第一问→第二问
        else if ((all_state & 0x000f) == 0x0002)
            all_state = 0x0003;  // 第二问→第三问
        else if ((all_state & 0x000f) == 0x0003)
            all_state = 0x0004;  // 第三问→第四问
        else if ((all_state & 0x000f) == 0x0004)
            all_state = 0x0001;  // 第四问→第一问（新增循环逻辑）
        break;                    
    
    case 2:    
        all_state = all_state | 0x0010;  // 保留原逻辑
        break;
}

	delay_ms (10);
	
	switch(all_state )
	{
		
//====================第一问状态=================================================
//===============================================================================		
		case 17:
				delay_ms (200);
			while (Get_Infrared_State()==0)
			{
				
				while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
				ack=1;
				
				float Yaw0_360;
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用无死区的pid_angle2
						int pid_out = pid_angle2(0, yaw_int);

						// 电机速度分配
						int aleft = 30 - pid_out;
						int aright = 30 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
			}				
			Set_Speed(0,0);
			LED1_Turn();
			buzzer_turn();

		break;
//===============================================================================			
//===============================================================================



			
//==============第二问状态=======================================================
//===============================================================================
		case 18:
				buzzer_off();
				LED1_ON ();
				delay_ms (100);
		while (1)
		{			
			switch (step )
			{
				delay_ms (500);
				case 1:
				{
					while (Get_Infrared_State()==0)
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							float Yaw0_360;
						
						if(Yaw < 0)         //第一次直线
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(0, yaw_int);    

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=2;                // 第四步：切换到循迹步骤
					}				
				}break;
			
				case 2:
				{   
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							
						    OLED_ShowSignedNum(1,1,Yaw,5);
							OLED_ShowBinNum(4,1,Get_Infrared_State(),8);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}		

					while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
					
					if (Get_Infrared_State()==0 )
					{
						LED1_Turn();
						buzzer_turn();		
						Set_Speed(0,0);
						delay_ms(200);					
						
					while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
					ack=1;

					step=3;
					
					
					}
					else step=2;
				
				}break;			
				
				case 3:{   

					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;			
						float Yaw0_360;             //第二次直线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用无死区的pid_angle2
						int pid_out = pid_angle2(168, yaw_int);

						// 电机速度分配
						int aleft = 30 - pid_out;
						int aright = 30 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}

					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=4;                // 第四步：切换到循迹步骤
					}
					else  step=3;
					}break;	
	
				case 4:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,25); 
					}
					if (Get_Infrared_State()==0)     step=5;// Set_Speed(18,18),delay_ms(4),
					else  step=4;
				}break;				
								
				
				case 5:{   
					
					LED1_Turn();
					buzzer_turn();					
					
					while(1)
					{
						OLED_ShowSignedNum(1,1,Yaw,5);
						Set_Speed(0,0);
	

						step=999;
			
					}
				}break;	
			}				
			}		
			
	
//===============================================================================
//===============================================================================		
	

			
//===================第三问======================================================
//===============================================================================
		case 19:
			  buzzer_off();
				LED1_ON ();
				delay_ms (10);
		while (1)
		{			
			switch (step )
			{
				case 1:
				{
					delay_ms (100);
					while (Get_Infrared_State()==0)
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						float Yaw0_360;
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(50, yaw_int);

						// 电机速度分配
						int aleft = 30 - pid_out;
						int aright = 30 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=2;                // 第四步：切换到循迹步骤
					}
				}break;
			
				case 2:
				{   
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}	
					LED1_Turn();
					buzzer_turn();					
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=3;                // 第四步：切换到斜线步骤
					}
				
				}break;			
				
				case 3:{  
	
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							float Yaw0_360;
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(130, yaw_int);

						// 电机速度分配
						int aleft = 30 - pid_out;
						int aright = 30 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=4;                // 第四步：切换到循迹步骤
					}
					}break;	
							
				
				
				case 4:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=5;                // 第四步：切换到循迹步骤
					}
				}break;				
								
				
				case 5:{   
					
					LED1_Turn();
					buzzer_turn();					
					
					while(1)
					{
						OLED_ShowSignedNum(1,1,Yaw,5);
						Set_Speed(0,0);
	

						step=999;
			
					}
				}break;	
			}				
			}	

//======================================================================================================
//======================================================================================================
						
			
			
			
			
			
			
//========================第四问=========================================================================
//=======================================================================================================

		case 20:
			  buzzer_off();
				LED1_ON ();
				delay_ms (10);
		while (1)
		{			
			switch (step )
			{
				case 1:
				{
					delay_ms (100);
					while (Get_Infrared_State()==0)
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						float Yaw0_360;
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(50, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=2;                // 第四步：切换到循迹步骤
					}
				}break;
			
				case 2:
				{   
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}	
					LED1_Turn();
					buzzer_turn();					
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=3;                // 第四步：切换到斜线步骤
					}
				
				}break;			
				
				case 3:{  
	
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							float Yaw0_360;
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(129, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=4;                // 第四步：切换到循迹步骤
					}
					}break;	
							
				
				
				case 4:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=5;                // 第四步：切换到循迹步骤
					}
				  }break;				
	
				
//======================================第二圈==========================================================
//======================================================================================================
				
				
				case 5:
				{
					delay_ms (300);
					while (Get_Infrared_State()==0)
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						float Yaw0_360;        //第一次斜线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(59, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(100);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=6;                // 第四步：切换到循迹步骤
					}
				  }break;
			
				case 6:
				{ 
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}	
					LED1_Turn();
					buzzer_turn();					
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=7;                // 第四步：切换到斜线步骤
					}
				  }break;			
				
				case 7:{  
	        delay_ms(300);
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							float Yaw0_360;         //第二次斜线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(137, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(100);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=8;                // 第四步：切换到循迹步骤
					}
					}break;	
							
				
				
				case 8:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=9;                // 第四步：切换到循迹步骤
					}
				  }break;
//======================================第三圈==========================================================
//======================================================================================================
				
				
				case 9:
				{
					delay_ms (300);
					while (Get_Infrared_State()==0)
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						float Yaw0_360;        //第一次斜线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(65, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(100);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=10;                // 第四步：切换到循迹步骤
					}
				  }break;
			
				case 10:
				{ 
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}	
					LED1_Turn();
					buzzer_turn();					
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=11;                // 第四步：切换到斜线步骤
					}
				  }break;			
				
				case 11:{  
	        delay_ms(300);
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							float Yaw0_360;         //第二次斜线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(145, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(100);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=12;                // 第四步：切换到循迹步骤
					}
					}break;	
							
				
				
				case 12:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=13;                // 第四步：切换到循迹步骤
					}
				  }break;				
					
//======================================第四圈==========================================================
//======================================================================================================
				
				
				case 13:
				{
					delay_ms (300);
					while (Get_Infrared_State()==0)
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						float Yaw0_360;        //第一次斜线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(71, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(100);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=14;                // 第四步：切换到循迹步骤
					}
				  }break;
			
				case 14:
				{ 
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}	
					LED1_Turn();
					buzzer_turn();					
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=15;                // 第四步：切换到斜线步骤
					}
				  }break;			
				
				case 15:{  
	        delay_ms(300);
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							float Yaw0_360;         //第二次斜线
						
						if(Yaw < 0)  
								Yaw0_360 = Yaw + 360;
						else       
								Yaw0_360 = Yaw;    
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用pid_angle2
						int pid_out = pid_angle2(150, yaw_int);

						// 电机速度分配
						int aleft = 25 - pid_out;
						int aright = 25 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Set_Speed(aleft, aright);
						OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if  (Get_Infrared_State()!=0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(100);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=16;                // 第四步：切换到循迹步骤
					}
					}break;	
							
				
				
				case 16:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if  (Get_Infrared_State()==0) 
					{
							Set_Speed(0, 0);       // 第一步：先让电机完全停车
							delay_ms(50);          // 第二步：延时，确保红外状态稳定、惯性消除
							PID_out(0, 0);         // 第三步：重置速度环last_err，此时无惯性干扰
						  find_err = 0.0f;
							step=17;                // 第四步：切换到循迹步骤
					}
				  }break;					

					
				case 17:{   
					
					LED1_Turn();
					buzzer_turn();					
					
					while(1)
					{
						OLED_ShowSignedNum(1,1,Yaw,5);
						Set_Speed(0,0);
						step=999;			
					}
				  }break;
				}
		 }				
  }	
 }
}