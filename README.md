# 2024XMCar — STM32F103 循迹小车

基于 STM32F103RCT6 的 PID 循迹小车，使用 MPU6050 陀螺仪做角度闭环控制。

## 硬件

| 模块 | 型号 | 接口 |
|------|------|------|
| MCU | STM32F103RCT6 | — |
| 陀螺仪 | MPU6050 | I2C PB6/PB7 |
| OLED | SSD1306 128×32 | I2C PB10/PB11 |
| 红外传感器 | 8路 TCRT5000 | GPIO |
| 电机驱动 | TB6612 / L298N | PWM TIM2 |
| 电源 | 7.4V 锂电池 | — |

## 引脚连接

### I2C 总线

| 总线 | SCL | SDA | 设备 | 地址 | 速率 |
|------|-----|-----|------|------|------|
| I2C1 | **PB6** | **PB7** | MPU6050 | 0x68 | 400kHz |
| I2C2 | **PB10** | **PB11** | SSD1306 OLED | 0x3C (7位) / 0x78 (8位) | 400kHz |

### MPU6050 六轴陀螺仪 (I2C1: PB6/PB7)

| MPU6050 引脚 | STM32 引脚 | 说明 |
|-------------|-----------|------|
| VCC | 3.3V | 供电 |
| GND | GND | 地 |
| SCL | **PB6** | I2C 时钟 |
| SDA | **PB7** | I2C 数据 |
| AD0 | GND | 地址选择 (接地=0x68) |
| INT | 不接 | DMP 轮询模式，不需要中断 |
| XDA/XCL | 不接 | 不使用外部磁力计 |

### SSD1306 OLED 128×32 (I2C2: PB10/PB11)

| OLED 引脚 | STM32 引脚 | 说明 |
|----------|-----------|------|
| VCC | 3.3V | 供电 |
| GND | GND | 地 |
| SCL | **PB10** | I2C 时钟 |
| SDA | **PB11** | I2C 数据 |

### 8 路红外循迹传感器

陀螺仪方向为 X 轴朝前。传感器按从左到右顺序排列：

| 编号 | STM32 引脚 | 状态字节位 | 位置 | 检测到线时 |
|------|-----------|-----------|------|-----------|
| TRACK1 | **PA15** | bit7 (MSB) | 最左 | HIGH |
| TRACK2 | **PA8** | bit6 | 左二 | HIGH |
| TRACK3 | **PA11** | bit5 | 左三 | HIGH |
| TRACK4 | **PB15** | bit4 | 中间偏左 | HIGH |
| TRACK5 | **PA7** | bit3 | 中间偏右 | HIGH |
| TRACK6 | **PB14** | bit2 | 右三 | HIGH |
| TRACK7 | **PA6** | bit1 | 右二 | HIGH |
| TRACK8 | **PB13** | bit0 (LSB) | 最右 | HIGH |

- 全部配置为 **输入上拉 (INPUT_PULLUP)**
- 检测到黑线 → 传感器输出 HIGH → 对应位 = 1
- 白底无黑线 → 传感器输出 LOW → 对应位 = 0
- `Get_Infrared_State()` 返回值 = 0x00 表示所有传感器都检测到线（机器人正对黑线）
- 返回值 = 0xFF 表示所有传感器都在白底上（机器人离线）

### 双路直流电机

**左电机：**

| 电机驱动引脚 | STM32 引脚 | 功能 | 正转时 | 反转时 |
|-------------|-----------|------|--------|--------|
| IN1 | **PA3** | 方向控制 | HIGH | LOW |
| IN2 | **PA4** | 方向控制 | LOW | HIGH |
| PWM | **PA2** (TIM2 CH3) | 速度控制 | analogWrite 0-255 | |

**右电机：**

| 电机驱动引脚 | STM32 引脚 | 功能 | 正转时 | 反转时 |
|-------------|-----------|------|--------|--------|
| IN1 | **PA0** | 方向控制 | HIGH | LOW |
| IN2 | **PA5** | 方向控制 | LOW | HIGH |
| PWM | **PA1** (TIM2 CH2) | 速度控制 | analogWrite 0-255 | |

- PWM 频率: ~1kHz (默认)
- 占空比映射: 速度值 0~50 → analogWrite 0~255
- 速度范围: [-50, 50]，负值表示反转
- PID 输出限制: 速度 5~40

### 按键与指示

| 外设 | STM32 引脚 | 模式 | 有效电平 | 说明 |
|------|-----------|------|---------|------|
| **LED** | **PB9** | 推挽输出 | **HIGH** = 亮 | 状态指示灯 |
| **蜂鸣器** | **PB8** | 推挽输出 | LOW = 响 | 提示音 |
| **Key1 (模式)** | **PB4** | 输入上拉 | **LOW** = 按下 | 循环切换 4 种模式 |
| **Key2 (启动)** | **PB3** | 输入上拉 | **LOW** = 按下 | 启动当前模式 |

> **注意**: PB3/PB4 默认是 JTAG 引脚，代码中已通过 `AFIO->MAPR` 禁用 JTAG（保留 SWD）以将 PB3/PB4 用作普通 GPIO。

### 完整引脚总表

| STM32 引脚 | 功能 | 方向 | 备注 |
|-----------|------|------|------|
| PA0 | 右电机 IN1 | OUTPUT | |
| PA1 | 右电机 PWM | OUTPUT | TIM2 CH2 |
| PA2 | 左电机 PWM | OUTPUT | TIM2 CH3 |
| PA3 | 左电机 IN1 | OUTPUT | |
| PA4 | 左电机 IN2 | OUTPUT | |
| PA5 | 右电机 IN2 | OUTPUT | |
| PA6 | TRACK7 | INPUT_PULLUP | |
| PA7 | TRACK5 | INPUT_PULLUP | |
| PA8 | TRACK2 | INPUT_PULLUP | |
| PA11 | TRACK3 | INPUT_PULLUP | |
| PA15 | TRACK1 | INPUT_PULLUP | |
| PB3 | Key2 (启动) | INPUT_PULLUP | 需禁用 JTAG |
| PB4 | Key1 (模式) | INPUT_PULLUP | 需禁用 JTAG |
| PB6 | MPU6050 SCL | I2C1 | |
| PB7 | MPU6050 SDA | I2C1 | |
| PB8 | 蜂鸣器 | OUTPUT | |
| PB9 | LED | OUTPUT | HIGH=亮 |
| PB10 | OLED SCL | I2C2 | |
| PB11 | OLED SDA | I2C2 | |
| PB13 | TRACK8 | INPUT_PULLUP | |
| PB14 | TRACK6 | INPUT_PULLUP | |
| PB15 | TRACK4 | INPUT_PULLUP | |

## 开发环境

- **框架**: PlatformIO + Arduino
- **编译**: `platformio run`
- **烧录**: `platformio run -t upload` (ST-Link)
- **原 Keil 项目** 在 `User/` 目录下保留作为参考

## 使用方法

1. 上电后 OLED 显示 `READY`，LED 常亮
2. **Key1 (模式)**: 循环切换 4 种运行模式
3. **Key2 (启动)**: 启动当前选中的模式

### 运行模式

| 模式 | all_state | 说明 |
|------|-----------|------|
| 1 | 0x11 | 角度锁定 0° 直行 |
| 2 | 0x12 | 直行(0°)→循迹→直行(168°)→循迹 |
| 3 | 0x13 | 直行(50°)→循迹→直行(130°)→循迹 |
| 4 | 0x14 | 4 圈复杂路线 |

### PID 参数

| 参数 | 值 | 用途 |
|------|-----|------|
| Kp / Kd | 2.5 / 3.5 | 循迹 PD |
| ka2p / ka2d | 0.5 / 3.5 | 角度 PD |
| 角度输出限幅 | ±20 | |
| 循迹输出限幅 | ±25 | |
| 电机速度范围 | 5 ~ 40 | |

## 已知问题

**DMP 未启用**: MPU6050 的 DMP（数字运动处理器）固件加载在 Arduino Wire 库下不稳定，当前使用原始陀螺仪 Z 轴积分计算偏航角。Yaw 会随时间和温度漂移，但短时间运行足够。

如需修复 DMP，可尝试：
1. 降低 I2C 速度至 100kHz
2. 在 `mpu6050_write()` 中添加微秒延时模拟原 bit-bang 时序

## 原项目

原 Keil MDK 项目代码保留在 `Hardware/`、`Library/`、`Start/`、`System/`、`User/` 目录中，完整审计文档见 `KEIL_PROJECT_AUDIT.md`。
