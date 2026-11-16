# STM32 双轮自平衡小车

基于 STM32F103C8（标准外设库，Keil MDK）的两轮自平衡小车。
控制周期 10ms，姿态由 MPU6050 互补滤波解算，电机为 TB6612 类 H 桥驱动。

## 1. 功能

- 直立环 PD + 速度环 PI + 转向环 P 三环叠加
- MPU6050 软件 I2C 读取，初始化时自动校准陀螺零偏
- 互补滤波（按 dt 归一化，时间常数约 0.5s）
- OLED 显示（1KB 显存 + 分页刷新，不阻塞控制周期）
- 电池电压采集与低压保护（连续 3 次低于 6.5V 判故障）
- 倾角保护（超过 ±45° 停机并锁存，需复位）
- 独立看门狗（IWDG，约 406ms，调试时冻结）
- 蓝牙 USART2 指令：`F` 前进、`B` 后退、`L/R` 转向、`S` 停止
- 超声波 HC-SR04 避障（代码完整，默认关闭，见第 7 节）

## 2. 控制结构

```
pitch, gyro_x ──► 直立环 PD ─┐
target_speed, 编码器平均速度 ──► 速度环 PI ──┼──► u ──┐
target_turn, gyro_z ──► 转向环 P ───────────┘        ├──► 左右轮 PWM（限幅 + 死区补偿）
                                                      ┘
```

- 直立环：`u_pd = KP × (pitch − zero) + KD × gyro`
- 速度环：`u_speed = PI(0 − speed_avg)`，输出限幅 ±2000
- 转向环：`turn = P(0 − gyro_z)`，输出限幅 ±2000
- 混合：`pwm_l = u + turn`，`pwm_r = u − turn`，再按最大幅值等比限幅

PID 实现细节（`Hardware/Pid.c`）：

- 微分先行（对测量值求导），避免目标突变冲击
- 一阶低通滤波，`d_alpha` 越接近 1 滤波越强
- 积分限幅 + 饱和时回退积分（抗积分饱和）
- 死区、输出限幅、作用方向（`direction`）可配置

## 3. 引脚分配

| 功能 | 引脚 | 备注 |
|---|---|---|
| 电机 A/B PWM | PA8 (TIM1_CH1)、PA11 (TIM1_CH4) | 10kHz |
| 电机 A 方向 | PB14 / PB15 | 推挽输出 |
| 电机 B 方向 | PB12 / PB13 | 推挽输出 |
| 编码器 A | PA6 / PA7 (TIM3) | 编码器模式 TI12 |
| 编码器 B | PB6 / PB7 (TIM4) | 编码器模式 TI12 |
| MPU6050 | PB10 (SCL) / PB11 (SDA) | 软件 I2C，开漏 |
| OLED | PB8 (SCL) / PB9 (SDA) | 软件 I2C，开漏 |
| 蓝牙 | PA2 (TX) / PA3 (RX) | USART2，9600 |
| 电池电压 | PA4 | ADC1_IN4，分压比 5.0 |
| 超声波 | PA0 (TRIG) / PA1 (ECHO) | TIM2_CH2 输入捕获 |

## 4. 关键参数（`Hardware/Balance.h`、`Balance.c`）

| 参数 | 值 | 说明 |
|---|---|---|
| `BALANCE_KP` | 80 | 直立环比例 |
| `BALANCE_KD` | 1.5 | 直立环微分（陀螺仪） |
| `BALANCE_ZERO_POINT` | 0 | 机械平衡点（需实测） |
| `PITCH_SIGN` | −1 | 姿态符号（按实际安装方向实测） |
| `MOTOR_DEADZONE` | 400 | 静摩擦补偿 |
| `SPEED_KP / KI` | 15 / 0.8 | 速度环 |
| `TURN_KP` | 20 | 转向环 |
| `FALL_ANGLE_THRESHOLD` | 45° | 倾倒保护 |

调参顺序建议：先关速度环和转向环，只调直立环 KP 到"能站但低频晃"，再加 KD 抑制晃动；
然后加速度环 KI 消除慢漂移；最后加转向环。

## 5. 代码结构

```
User/main.c          主循环：10ms 控制拍、显示调度、蓝牙、保护逻辑
User/stm32f10x_it.c  异常向量
Hardware/Balance.c   三环计算、电机输出、故障处理
Hardware/Pid.c       PID 实现
Hardware/Mpu6050.c   软件 I2C、初始化校准、互补滤波
Hardware/Encoder.c   TIM3/TIM4 编码器接口
Hardware/Motor.c     TIM1 PWM + 方向控制
Hardware/OLED.c      SSD1306 驱动（显存 + 分页刷新）
Hardware/ADC.c       电池电压采集
Hardware/Bluetooth.c USART2 环形缓冲 + 指令
Hardware/Hcsr04.c    超声波测距（输入捕获）
System/Delay.c       基于 SysTick 的 us/ms/s 延时
```

## 6. 编译与烧录

1. Keil 打开 `project.uvprojx`
2. `Rebuild`（F7）：当前 0 Error / 0 Warning，Flash 15.6KB，RAM 3.9KB
3. ST-Link 连接 SWD（GND/SWDIO/SWCLK），按 `F8` 下载
4. 上电后保持车身静止约 2 秒（陀螺仪零偏校准），屏幕显示 `BALANCING`

## 7. 调试记录与已知限制

- **编码器**：当前样机 PA6/PA7、PB6/PB7 上测不到计数（引脚已改为内部上拉后仍只有 ±1 噪声），
  判断为编码器供电/线序或器件问题，速度环因此未完成闭环验证。
- **超声波**：触发脉宽与量程判断已修正，但默认 `ENABLE_SONAR = 0`，需实测后再启用。
- **Key/LED**：引脚与 MPU6050（PB11）和超声波/串口（PA1/PA2）冲突，当前未启用。
- **姿态符号**：`PITCH_SIGN = -1`、转向环方向 `-1` 均由实车现象反推确定（原方向为正反馈）。
