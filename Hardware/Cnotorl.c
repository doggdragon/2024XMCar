#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Timer.h"
#include "TRACK.h"

float error;

#define Kp    2.5
#define Kd    3.5
  
u8 last_infrared_state = 0;	
	
float Track_err(void)
{
	u8 state =  Get_Infrared_State();
	// 优化：新增“连续2次状态一致才更新误差”，既防抖又避免更新滞后
	static u8 same_state_cnt = 0; // 新增：连续相同状态计数器
	
	if (state == last_infrared_state)
	{
	    same_state_cnt++; // 状态一致，计数器加1
	}
	else
	{
	    same_state_cnt = 0; // 状态不一致，计数器清零
	    last_infrared_state = state;
	}
	
	// 仅当连续2次状态一致时，才更新error，否则返回上一次误差
	if (same_state_cnt < 2)
	{
	    return error;
	}
	
	switch(state)
	{  
		case 0:     //0000 0000   
		error= 0 ;	  break;
		case 16:    //0001 0000   
		error= 1 ;  break;
		case 8:     //0000 1000   
		error= 1 ; break;	
		case 24:    //0001 1000   
		error= 0 ;	  break;		
		case 60:    //0011 1100   
		error= 0 ;	  break;		
		case 126:   //0111 1110   
		error= 0 ;	  break;
		
		case 48:     //0011 0000   //小车右偏，err为正
		error= 2  ;	  break;
		case 32:     //0010 0000   
		error= 2 ;	  break;
		case 64:     //0100 0000   
		error= 4 ;	  break;
		case 96:     //0110 0000   
		error= 4 ;	  break;
		case 128:    //1000 0000   
		error= 6 ;	  break;	
		case 192:    //1100 0000   
		error= 6 ;	  break;
		case 224:   //1110 0000   
		error= 6 ;	  break;
		case 160:   //1010 0000   
		error= 6 ;	  break;
		case 254:   //1111 1110   
		error= 6 ;	  break;
		case 252:   //1111 1100   
		error= 6 ;	  break;
		case 248:   //1111 1000   
		error= 6 ;	  break;
		case 240:   //1111 0000   
		error= 6 ;	  break;
		case 124:   //0111 1100   
		error= 4 ;	  break;
		case 120:   //0111 1000   
		error= 4 ;	  break;
		case 56:    //0011 1000   
		error= 2 ;	break;
		
		
		

		case 12:   //0000 1100   //小车左偏，err为负
		error= -2 ;	  break;	
    case 14:   //0000 1110   //小车左偏，err为负
		error= -4 ;	  break;	
    case 30:   //0001 1110   //小车左偏，err为负
		error= -4 ;	  break;
    case 62:   //0011 1110   //小车左偏，err为负
		error= -4 ;	  break;
		case 31:   //0001 1111   
		error= -6 ;	  break;
		case 63:   //0011 1111   
		error= -6 ;	  break;
		case 125:  //0111 1111   
		error= -6 ;	  break;
		case 4:    //0000 0100   
		error= -2 ;	  break;	
    case 28:   //0001 1100   
		error= -2 ;	  break;		
		case 2:    //0000 0010   
		error= -4 ;	  break;
		case 6:    //0000 0110   
		error= -4 ;	  break;
		case 1:    //0000 0001   
		error= -6 ;	  break;
		case 3:    //0000 0011   
		error= -6 ;	  break;
		case 7:    //0000 0111   
		error= -6 ;	  break;
		case 15:   //0000 1111   
		error= -6 ;	  break;
			
		default: 
		// 优化点1：默认状态不重置error为0，沿用当前值，避免无效跳变
		break;
	}
	last_infrared_state = state; 
	same_state_cnt = 0; // 重置计数器，为下一次防抖做准备
	return error;
}


int PID_out(float error, int Target)      //位置环
{
    static int last_err = 0;  // 保留原有静态变量，不新增全局变量
    
    int err = (int)error;
    int out;
    
    // 优化点3：抑制微分项初始跳变（角度环切换后首次循迹，避免输出突变）
    int diff = err - last_err;
    if (last_err == 0 && err != 0) {
        diff = err / 2;  // 微分项减半，减少初始冲击
    }
    
    // 保留原有PD计算逻辑，仅替换微分项为优化后的diff
    out = Kp * err + Kd * diff;
    
    // 优化点4：增加极简输出限幅，避免超出电机有效范围
    if (out > 25) out = 25;    // 上限从20提升到25，释放输出空间
    if (out < -25) out = -25;  // 下限对称提升，保持转向平衡
    
    last_err = err;  // 保留原有历史误差更新
    return out;
}


void Final_Speed(int pid_out ,int base_speed)
{
    int left_speed = base_speed - pid_out;
    int right_speed = base_speed + pid_out;
    
    // 软过渡：限制电机速度变化幅度，避免爆冲
    #define MIN_SPEED 5
    #define MAX_SPEED 40
    
    // 左电机限幅
    left_speed = left_speed < MIN_SPEED ? MIN_SPEED : left_speed;
    left_speed = left_speed > MAX_SPEED ? MAX_SPEED : left_speed;
    // 右电机限幅
    right_speed = right_speed < MIN_SPEED ? MIN_SPEED : right_speed;
    right_speed = right_speed > MAX_SPEED ? MAX_SPEED : right_speed;
    
    Set_Speed(left_speed, right_speed);
}


// 保留原有PD参数定义（可根据调试微调）
#define  ka2p   0.5
#define  ka2d   3.5

int pid_angle2(int target, int yaw)     //角度环
{
    static int err_last = 0;  // 初始化历史误差（静态变量，记录上一次的有效误差）
    int pid_a_out, err;
  
    // 第一步：将原始yaw（±180°）转换为0~360°，统一角度坐标系
    int yaw0_360 = (yaw < 0) ? (yaw + 360) : yaw;
    int target_360 = target;  // 目标角度转为0~360°（输入target需为0~360°或±180°，此处兼容）

    // 第二步：重构误差计算逻辑
    // 核心修改：计算「最短路径误差」，而非强制逆时针优先
    // 误差范围限定在[-180, 180]，避免出现358°这类超大误差，从根源上防止转圈
    err = target_360 - yaw0_360;
    // 若误差大于180°，取反向误差（走更短路径，避免逆时针超大误差）
    if (err > 180) {
        err -= 360;
    }
    // 若误差小于-180°，同样取反向误差（保持最短路径）
    else if (err < -180) {
        err += 360;
    }

    // 第三步：优化微分项计算，避免首次跳变和累积误差
    int diff = err - err_last;
    // 首次调用时（err_last=0且err≠0），微分项减半
    if (err_last == 0 && err != 0) {
        diff = err / 2;
    }

    // 第四步：PD计算（无积分项，无死区，纯PD输出）
    // 直接通过比例项+微分项得到修正量，无额外闭环限制
    pid_a_out = (int)(ka2p * err + ka2d * diff);  // 强制转整型，匹配后续电机控制

    // 第五步：限制PID输出最大值，避免超大修正量导致失控转圈
    // 无死区情况下，限幅是防止转圈的关键（避免修正量过大导致越过目标后无法收敛）
    #define PID_A_OUT_MAX 20  // 可根据实际调试调整（建议15~25）
    if (pid_a_out > PID_A_OUT_MAX) {
        pid_a_out = PID_A_OUT_MAX;
    }
    if (pid_a_out < -PID_A_OUT_MAX) {
        pid_a_out = -PID_A_OUT_MAX;
    }

    // 第六步：更新历史误差，为下一次计算做准备（无重置，纯自然累积）
    err_last = err;

    // 返回PD修正量，无死区闭环，依赖PD参数和限幅实现自然收敛
    return pid_a_out;
}

