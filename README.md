# STM32 平衡车

基于 STM32F103C8 的双轮自平衡小车，Keil 标准库开发，控制周期 10ms。

代码参考了开发板例程和芯片手册，按模块拆开写，参数集中在 `Hardware/Balance.h`，方便后面继续调。

## 硬件配置

- STM32F103C8 最小系统
- MPU6050 姿态传感器，软件 I2C
- TB6612 电机驱动，两个直流减速电机
- 两个编码器，TIM3 / TIM4 编码器模式
- SSD1306 OLED，软件 I2C
- HC-05 蓝牙模块，USART2，9600
- 电池电压检测，PA4 接 ADC1_IN4
- HC-SR04 超声波，代码已预留

引脚分配：

| 功能 | 引脚 | 说明 |
|---|---|---|
| 电机 PWM | PA8、PA11 | TIM1_CH1、TIM1_CH4，10kHz |
| 电机方向 | PB14/PB15、PB12/PB13 | 推挽输出 |
| 编码器 A | PA6/PA7 | TIM3 编码器模式 |
| 编码器 B | PB6/PB7 | TIM4 编码器模式 |
| MPU6050 | PB10/SCL、PB11/SDA | 软件 I2C，开漏 |
| OLED | PB8/SCL、PB9/SDA | 软件 I2C，开漏 |
| 蓝牙 | PA2/TX、PA3/RX | USART2 |
| 电池检测 | PA4 | 分压比 5.0 |
| 超声波 | PA0/TRIG、PA1/ECHO | TIM2 输入捕获 |

## 控制结构

直立环 PD、速度环 PI、转向环 P，三个环的输出相加后给左右轮。

```
pitch、gyro_x        → 直立环 PD ┐
target_speed、speed  → 速度环 PI ┼→ u ─┐
target_turn、gyro_z  → 转向环 P  ┘      ├→ 左右轮 PWM
                                        ┘
```

直立环的微分项直接用陀螺仪，避免对角度求导带来的噪声。MPU6050 用互补滤波，按 dt 归一化，时间常数约 0.5s。

主要参数：

| 参数 | 值 | 说明 |
|---|---|---|
| `BALANCE_KP` | 80 | 直立环比例 |
| `BALANCE_KD` | 1.5 | 直立环微分 |
| `BALANCE_ZERO_POINT` | 0 | 机械平衡点 |
| `PITCH_SIGN` | -1 | 姿态方向 |
| `MOTOR_DEADZONE` | 400 | 电机死区补偿 |
| `SPEED_KP` / `SPEED_KI` | 15 / 0.8 | 速度环 |
| `TURN_KP` | 20 | 转向环 |
| `FALL_ANGLE_THRESHOLD` | 45° | 倾倒保护 |

## 代码结构

```
User/main.c          主循环，10ms 控制拍
User/stm32f10x_it.c  中断向量
Hardware/Balance.c   三环计算、电机输出、保护逻辑
Hardware/Pid.c       PID 实现
Hardware/Mpu6050.c   软件 I2C、零偏校准、互补滤波
Hardware/Encoder.c   TIM3/TIM4 编码器
Hardware/Motor.c     TIM1 PWM 和方向控制
Hardware/OLED.c      SSD1306 驱动
Hardware/ADC.c       电池电压采集
Hardware/Bluetooth.c USART2 指令解析
Hardware/Hcsr04.c    超声波测距
System/Delay.c       SysTick 延时
```

## 编译与烧录

1. Keil 打开 `project.uvprojx`
2. F7 编译，当前 0 Error / 0 Warning
3. ST-Link 接 SWD：GND、SWDIO、SWCLK
4. F8 下载，上电后保持车身静止约 2 秒，等陀螺仪零偏校准完成

## 说明

目前已经能跑，参数会根据实车情况再改。

项目还在完善中，代码和文档后面会继续更新。
