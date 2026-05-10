# Keil5 项目完整审计：STM32F103 循迹小车

> 基于 `D:\Code\2024H+32` 所有 `.c`/`.h` 源码，逐行提取。

---

## 1. 完整引脚映射

### 红外循迹传感器 (TRACK.h + TRACK.c)
| 编号 | GPIO | 引脚 | 8位状态中的位 |
|------|------|------|-------------|
| TRACK1 | GPIOA | **PA15** | bit7 (MSB) |
| TRACK2 | GPIOA | **PA8** | bit6 |
| TRACK3 | GPIOA | **PA11** | bit5 |
| TRACK4 | GPIOB | **PB15** | bit4 |
| TRACK5 | GPIOA | **PA7** | bit3 |
| TRACK6 | GPIOB | **PB14** | bit2 |
| TRACK7 | GPIOA | **PA6** | bit1 |
| TRACK8 | GPIOB | **PB13** | bit0 (LSB) |
- 全部: `INPUT_PULLUP`, `Get_Infrared_State()` 返回 8 位(1=检测到线)

### 电机 (Motor.c + PWM.h)
| 功能 | GPIO | 引脚 |
|------|------|------|
| 左电机方向 IN1 | GPIOA | **PA3** |
| 左电机方向 IN2 | GPIOA | **PA4** |
| 右电机方向 IN1 | GPIOA | **PA0** |
| 右电机方向 IN2 | GPIOA | **PA5** |
| 左电机 PWM | GPIOA | **PA2** (TIM2 CH3) |
| 右电机 PWM | GPIOA | **PA1** (TIM2 CH2) |
- PWM 参数: TIM2, ARR=99, PSC=35, 20kHz, 范围 [-50, 50]

### LED (LED.c)
| 功能 | GPIO | 引脚 | 有效电平 |
|------|------|------|---------|
| LED1 | GPIOB | **PB9** | **低电平亮** |

### 蜂鸣器 (buzzer.c)
| 功能 | GPIO | 引脚 | 有效电平 |
|------|------|------|---------|
| Buzzer | GPIOB | **PB8** | **低电平响** |

### 按键 (Key.c)
| 按键 | GPIO | 引脚 | 返回值 | 备注 |
|------|------|------|--------|------|
| Key1 | GPIOB | **PB4** | 1 | 切换模式 |
| Key2 | GPIOB | **PB3** | 2 | 启动运行 |
- 输入上拉, 低电平有效, 20ms 消抖
- **Key_Init() 会禁用 JTAG** 以释放 PB3/PB4

### OLED (OLED.c — 软件 I2C 位打击)
| 功能 | GPIO | 引脚 |
|------|------|------|
| SCL | GPIOB | **PB10** |
| SDA | GPIOB | **PB11** |
- I2C 地址: **0x78** (8位), 即 7位 0x3C
- 开漏输出, 无 ACK 检测

### MPU6050 (MPU6050_I2C.h)
| 功能 | GPIO | 引脚 |
|------|------|------|
| SCL | GPIOB | **PB6** |
| SDA | GPIOB | **PB7** |
- I2C 地址: **0x68**
- 软件 I2C 位打击, 4us 延时, 有 ACK 超时(250次)

---

## 2. I2C 总线架构 (重要!)

```
PB6 (SCL) + PB7 (SDA)  →  MPU6050 (0x68)    独立 I2C 总线 #1
PB10 (SCL) + PB11 (SDA) →  OLED (0x78/0x3C)  独立 I2C 总线 #2
```

**两条 I2C 总线完全独立，不共享！**

---

## 3. 所有常量

| 常量 | 值 | 用途 |
|------|-----|------|
| `PWM_MAX` | **50** | 电机 PWM 上限 |
| `PWM_MIN` | **-50** | 电机 PWM 下限 |
| `Kp` | **2.5** | 循迹 PD 比例系数 |
| `Kd` | **3.5** | 循迹 PD 微分系数 |
| `ka2p` | **0.5** | 角度 PD 比例系数 |
| `ka2d` | **3.5** | 角度 PD 微分系数 |
| `PID_A_OUT_MAX` | **20** | 角度环输出限幅 |
| `MIN_SPEED` | **5** | Final_Speed 最低速度 |
| `MAX_SPEED` | **40** | Final_Speed 最高速度 |
| PID_out 限幅 | **[-25, 25]** | 循迹环输出 |
| `DEFAULT_MPU_HZ` | **100** | DMP 采样率 |

---

## 4. 初始化顺序 (main.c 原始顺序)

```c
int main(void) {
    Track_Init();        // 1. 红外传感器
    LED_Init();          // 2. LED (PB9 HIGH=灭)
    buzzer_Init();       // 3. 蜂鸣器 (PB8 HIGH=关)
    Motor_Init();        // 4. 电机方向 GPIO + PWM (TIM2)
    Key_Init();          // 5. 按键 (禁用JTAG)
    OLED_Init();         // 6. OLED (PB10/PB11 位打击I2C)
    MPU6050_Init();      // 7. MPU6050 基础配置 (PB6/PB7)
    MPU6050_DMP_Init();  // 8. DMP 固件加载
    while(1) { ... }     // 9. 主循环
}
```

---

## 5. 主循环逻辑

### 默认状态 (all_state=0)
- 灭蜂鸣器, 亮LED
- 读取 DMP 数据 (阻塞等待)
- OLED 显示: Yaw, all_state(Hex), step, 红外状态(Bin)
- 检测按键

### 按键逻辑
- **Key1 (PB4)**: 循环切换低4位: 0→1→2→3→4→1...
- **Key2 (PB3)**: 置位 bit4 (`all_state |= 0x10`)，启动当前模式

### 状态机

| all_state | 说明 | 步骤 |
|-----------|------|------|
| **0x11 (17)** | 模式1: 简单直行 | 角度锁0°, 遇线停止 |
| **0x12 (18)** | 模式2: 直行+循迹 | 直行(0°)→循迹→直行(168°)→循迹→停 |
| **0x13 (19)** | 模式3: 直行+循迹 | 直行(50°)→循迹→直行(130°)→循迹→停 |
| **0x14 (20)** | 模式4: 4圈复杂 | 每圈: 直行(Ta)→循迹→直行(Tb)→循迹 |
| | | Lap1: 50°/129°, Lap2: 59°/137° |
| | | Lap3: 65°/145°, Lap4: 71°/150° |

### 模式4 角度目标一览

| 圈 | 直行目标1 | 直行目标2 | 循迹速度1 | 循迹速度2 |
|----|----------|----------|----------|----------|
| 1 | 50° | 129° | 25 | 22 |
| 2 | 59° | 137° | 25 | 22 |
| 3 | 65° | 145° | 25 | 22 |
| 4 | 71° | 150° | 25 | 22 |

---

## 6. PID 算法详情

### Track_err() — 红外循迹误差
- 输入: 8位传感器状态
- 输出: float, 范围 [-6, +6]
- 消抖: 连续2次相同读数才更新
- 共40+个 case 覆盖各种传感器组合

### PID_out(error, target) — 循迹 PD 控制器
- `out = Kp*err + Kd*diff` (PD, 无积分)
- 首次非零误差: diff 减半 (软启动)
- 输出钳位: [-25, 25]

### pid_angle2(target, yaw) — 角度 PD 控制器
- 将 yaw 归一到 [0, 360)
- 计算最短路径误差 [-180, 180]
- `out = ka2p*err + ka2d*diff`
- 输出钳位: [-20, 20]

### Final_Speed(pid_out, base_speed)
- left = base_speed - pid_out
- right = base_speed + pid_out
- 钳位到 [5, 40]

---

## 7. Set_Speed 电机驱动逻辑

```c
// 左电机 (PA3=IN1, PA4=IN2)
if (motor_l >= 0)  PA3=HIGH, PA4=LOW, PWM_CH3 = motor_l
else               PA4=HIGH, PA3=LOW, PWM_CH3 = -motor_l

// 右电机 (PA0=IN1, PA5=IN2)  
if (motor_r >= 0)  PA0=HIGH, PA5=LOW, PWM_CH2 = motor_r
else               PA5=HIGH, PA0=LOW, PWM_CH2 = -motor_r
```

---

## 8. 与本重构相关的注意事项

1. **两条 I2C 是分开的** — 不能合并
2. **LED/蜂鸣器是低电平有效**
3. **红外传感器 `Get_Infrared_State()` 返回 1=检测到线** (HIGH=线)
4. **按键需禁用 JTAG** — Key_Init() 调用 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)`
5. **内存/Flash**: STM32F103RCT6 = 256KB Flash, 48KB RAM
