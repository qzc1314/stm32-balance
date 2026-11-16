#include "stm32f10x.h"                  // Device header

#include "delay.h"
#include "Mpu6050.h"
#include <math.h>

Attitude_t attitude;

// 软件I2C引脚定义 对应PB10-SCL PB11-SDA
#define MPU_SCL_PORT    GPIOB
#define MPU_SCL_PIN     GPIO_Pin_10
#define MPU_SDA_PORT    GPIOB
#define MPU_SDA_PIN     GPIO_Pin_11

#define MPU_ADDR        0xD0    // MPU6050设备地址
#define MPU_WHO_AM_I    0x75
#define MPU_ID_VALUE    0x68    // 正品 MPU6050；兼容芯片可能是 0x70/0x71/0x73/0x98
#define ALPHA           0.98f   // 互补滤波系数

// 陀螺仪零偏（初始化时自动校准）
static float gyro_x_offset = 0;
static float gyro_z_offset = 0;
static uint8_t mpu_ready = 0;
static uint8_t mpu_id = 0;      /* WHO_AM_I 读到的原始值，用于诊断 */

// ==================== 软件I2C底层时序 ====================
static void I2C_Delay(void)
{
    uint16_t i = 20;
    while(i--);
}

static void I2C_SCL_Set(uint8_t level)
{
    if(level) GPIO_SetBits(MPU_SCL_PORT, MPU_SCL_PIN);
    else GPIO_ResetBits(MPU_SCL_PORT, MPU_SCL_PIN);
}

static void I2C_SDA_Set(uint8_t level)
{
    if(level) GPIO_SetBits(MPU_SDA_PORT, MPU_SDA_PIN);
    else GPIO_ResetBits(MPU_SDA_PORT, MPU_SDA_PIN);
}

static uint8_t I2C_SDA_Read(void)
{
    return GPIO_ReadInputDataBit(MPU_SDA_PORT, MPU_SDA_PIN);
}

static void I2C_Start(void)
{
    I2C_SDA_Set(1);
    I2C_SCL_Set(1);
    I2C_Delay();
    I2C_SDA_Set(0);
    I2C_Delay();
    I2C_SCL_Set(0);
}

static void I2C_Stop(void)
{
    I2C_SDA_Set(0);
    I2C_SCL_Set(1);
    I2C_Delay();
    I2C_SDA_Set(1);
}

/* 总线恢复：如果上一次通讯被复位/掉电打断，从机可能一直拉住 SDA，
 * 后续所有读都会失败。补 9 个 SCL 让从机把剩余位吐完，再发一个 STOP。 */
static void I2C_BusRecover(void)
{
    uint8_t i;

    I2C_SDA_Set(1);
    for (i = 0; i < 9; i++)
    {
        I2C_SCL_Set(0);
        I2C_Delay();
        I2C_SCL_Set(1);
        I2C_Delay();
    }
    I2C_Stop();
}

static uint8_t I2C_WaitAck(void)
{
    uint8_t ack;
    I2C_SDA_Set(1);
    I2C_Delay();
    I2C_SCL_Set(1);
    I2C_Delay();
    ack = !I2C_SDA_Read();
    I2C_SCL_Set(0);
    return ack;
}

static void I2C_SendByte(uint8_t data)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        I2C_SDA_Set(data & 0x80);
        data <<= 1;
        I2C_Delay();
        I2C_SCL_Set(1);
        I2C_Delay();
        I2C_SCL_Set(0);
    }
    I2C_WaitAck();
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i, data = 0;
    I2C_SDA_Set(1);
    for(i=0; i<8; i++)
    {
        data <<= 1;
        I2C_SCL_Set(1);
        I2C_Delay();
        if(I2C_SDA_Read()) data |= 1;
        I2C_SCL_Set(0);
        I2C_Delay();
    }
    if(ack) I2C_SDA_Set(0);
    else I2C_SDA_Set(1);
    I2C_Delay();
    I2C_SCL_Set(1);
    I2C_Delay();
    I2C_SCL_Set(0);
    return data;
}

// ==================== MPU6050寄存器操作 ====================
static void MPU_WriteReg(uint8_t reg, uint8_t data)
{
    I2C_Start();
    I2C_SendByte(MPU_ADDR);
    I2C_SendByte(reg);
    I2C_SendByte(data);
    I2C_Stop();
}

static void MPU_ReadLen(uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    I2C_Start();
    I2C_SendByte(MPU_ADDR);
    I2C_SendByte(reg);
    I2C_Start();
    I2C_SendByte(MPU_ADDR | 1);
    for(i=0; i<len-1; i++)
        buf[i] = I2C_ReadByte(1);
    buf[i] = I2C_ReadByte(0);
    I2C_Stop();
}

static uint8_t MPU_ReadReg(uint8_t reg)
{
    uint8_t data = 0xFF;
    MPU_ReadLen(reg, 1, &data);
    return data;
}

// ==================== 初始化与姿态解算 ====================
/**
 * @brief  陀螺仪零偏校准（上电静止时执行）
 */
static void Gyro_Calibrate(void)
{
    uint16_t i;
    int32_t sum_x = 0, sum_z = 0;
    uint8_t buf[6];

    for(i=0; i<500; i++)
    {
        MPU_ReadLen(0x43, 6, buf);
        sum_x += (int16_t)((buf[0]<<8) | buf[1]);
        sum_z += (int16_t)((buf[4]<<8) | buf[5]);
        I2C_Delay();
    }
    gyro_x_offset = (float)sum_x / 500.0f / 16.4f;
    gyro_z_offset = (float)sum_z / 500.0f / 16.4f;
}

/**
 * @brief  MPU6050初始化
 * @retval 无；初始化失败时 MPU6050_IsReady() 返回 0
 */
void MPU6050_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    uint8_t buf[6];
    int16_t acc_y, acc_z;

    mpu_ready = 0;

    // 初始化GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Pin   = MPU_SCL_PIN | MPU_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;  // 开漏输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_SCL_Set(1);
    I2C_SDA_Set(1);
    I2C_BusRecover();

    // MPU6050寄存器配置
    MPU_WriteReg(0x6B, 0x80);   /* 复位 */
    Delay_ms(100);              /* 手册建议约 100ms */
    /* 0x03：PLL 以 Z 轴陀螺为参考。开启 DLPF 后手册推荐用 Z 参考，
     * 原来写的 0x01（X 轴）会让采样时钟与滤波通带不匹配。 */
    MPU_WriteReg(0x6B, 0x03);   // 解除休眠
    MPU_WriteReg(0x19, 0x00);   // 采样率分频
    MPU_WriteReg(0x1A, 0x03);   // 低通滤波 44Hz
    MPU_WriteReg(0x1B, 0x18);   // 陀螺仪量程 ±2000°/s
    MPU_WriteReg(0x1C, 0x08);   // 加速度计量程 ±4g

    /* 确认芯片在线。注意：便宜的模块常常是 MPU6500/MPU9250 等兼容芯片，
     * WHO_AM_I 不是 0x68（常见 0x70/0x71/0x73/0x98），但寄存器行为一致。
     * 因此只把 0x00/0xFF（总线无响应）判为故障，其余 ID 一律接受。 */
    mpu_id = MPU_ReadReg(MPU_WHO_AM_I);
    if (mpu_id == 0x00 || mpu_id == 0xFF)
    {
        I2C_BusRecover();               /* 再试一次，排除偶发总线卡死 */
        mpu_id = MPU_ReadReg(MPU_WHO_AM_I);
        if (mpu_id == 0x00 || mpu_id == 0xFF)
        {
            return;                     /* 确实没有芯片 */
        }
    }

    // 校准陀螺仪零偏
    Gyro_Calibrate();

    // 初始角度用加速度计计算
    MPU_ReadLen(0x3B, 6, buf);
    acc_y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    acc_z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    attitude.pitch = atan2f((float)acc_y, (float)acc_z) * 57.2957795f;

    mpu_ready = 1;
}

uint8_t MPU6050_IsReady(void)
{
    return mpu_ready;
}

uint8_t MPU6050_GetId(void)
{
    return mpu_id;
}

/**
 * @brief  读取数据并更新姿态（互补滤波），必须10ms调用一次
 */
void MPU6050_GetData(float dt)
{
    uint8_t buf[14];
    int16_t acc_y, acc_z;
    int16_t gyro_x_raw, gyro_z_raw;
    float pitch_acc;

    if (!mpu_ready) return;

    /* 拦住 0 / 负数 / NaN；超出合理范围退回 10ms */
    if (!(dt > 0.005f)) dt = 0.01f;
    if (dt > 0.05f)     dt = 0.01f;

    MPU_ReadLen(0x3B, 14, buf);

    acc_y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    acc_z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    gyro_x_raw = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    gyro_z_raw = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    attitude.gyro_x = (float)gyro_x_raw / 16.4f - gyro_x_offset;
    attitude.gyro_z = (float)gyro_z_raw / 16.4f - gyro_z_offset;

    pitch_acc = atan2f((float)acc_y, (float)acc_z) * 57.2957795f;

    /* 互补滤波按 dt 归一化：系数随采样周期变化，抖动时时间常数不会漂移。
     * alpha = 1 - dt/(dt + TAU)，TAU≈0.5s。 */
    {
        const float tau = 0.5f;
        float alpha = 1.0f - dt / (dt + tau);
        attitude.pitch = alpha * (attitude.pitch + attitude.gyro_x * dt)
                       + (1.0f - alpha) * pitch_acc;
    }
}
