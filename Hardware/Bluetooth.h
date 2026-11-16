#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f10x.h"

// 环形缓冲区大小（根据指令长度调整，典型值 64~256）
#define BT_RX_BUF_SIZE 64

void Bluetooth_Init(void);

// ----- 推荐使用的新接口 -----
uint8_t Bluetooth_ReadByte(uint8_t *pData);   // 从缓冲区读一个字节，返回 1 成功，0 缓冲区空
void Bluetooth_SendByte(uint8_t data);        // 发送一个字节
void Bluetooth_SendString(const char *str);   // 发送字符串
uint16_t Bluetooth_Available(void);           // 返回缓冲区中未读取的字节数

// ----- 保留旧接口（兼容，但不推荐）-----
// 注意：该函数会从缓冲区取一个字节，若无数据返回 0（可能与有效数据 0 混淆）
uint8_t Bluetooth_GetCmd(void);

#endif
