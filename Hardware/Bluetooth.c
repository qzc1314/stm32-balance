#include "stm32f10x.h"                  // Device header
#include "Bluetooth.h"

#define BT_USART    USART2

// ===================== 环形缓冲区 =====================
static uint8_t bt_rx_buf[BT_RX_BUF_SIZE];
static volatile uint8_t bt_rx_head = 0;   // 写入索引
static volatile uint8_t bt_rx_tail = 0;   // 读取索引

// ===================== 初始化 =====================
void Bluetooth_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // PA2 TX 复用推挽输出
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA3 RX 浮空输入
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 串口配置：9600波特率，8位数据，无校验，1停止位
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(BT_USART, &USART_InitStruct);

    // 开启接收中断
    USART_ITConfig(BT_USART, USART_IT_RXNE, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(BT_USART, ENABLE);
}

// ===================== 中断服务 =====================
void USART2_IRQHandler(void)
{
    // 接收中断
    if(USART_GetITStatus(BT_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(BT_USART);   // 读 DR 并清 RXNE
        uint8_t next_head = (bt_rx_head + 1) % BT_RX_BUF_SIZE;
        if(next_head != bt_rx_tail)                   // 缓冲区未满才存入
        {
            bt_rx_buf[bt_rx_head] = data;
            bt_rx_head = next_head;
        }
        // 若满则丢弃，可根据需要添加错误计数
        USART_ClearITPendingBit(BT_USART, USART_IT_RXNE);
    }

    // 溢出错误处理（必须读 DR 和 SR 来清除）
    if(USART_GetFlagStatus(BT_USART, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(BT_USART);   // 读取 DR
        // 其他错误标志（FE/NE）可根据需要处理，此处简单略过
        USART_ClearFlag(BT_USART, USART_FLAG_ORE);
    }
}

// ===================== 新接口实现 =====================

/**
 * @brief  从环形缓冲区读取一个字节
 * @param  pData: 存储读取到的数据
 * @retval 1 成功读取，0 缓冲区空
 */
uint8_t Bluetooth_ReadByte(uint8_t *pData)
{
    if(bt_rx_head == bt_rx_tail)
        return 0;                  // 缓冲区空
    *pData = bt_rx_buf[bt_rx_tail];
    bt_rx_tail = (bt_rx_tail + 1) % BT_RX_BUF_SIZE;
    return 1;
}

/**
 * @brief  发送一个字节（阻塞式）
 */
void Bluetooth_SendByte(uint8_t data)
{
    while(USART_GetFlagStatus(BT_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(BT_USART, data);
}

/**
 * @brief  发送字符串（以 '\0' 结尾）
 */
void Bluetooth_SendString(const char *str)
{
    while(*str)
    {
        Bluetooth_SendByte((uint8_t)*str);
        str++;
    }
}

/**
 * @brief  返回缓冲区中尚未读取的字节数
 */
uint16_t Bluetooth_Available(void)
{
    // 环形缓冲区有效数据量计算（假设不会完全填满）
    return (bt_rx_head - bt_rx_tail + BT_RX_BUF_SIZE) % BT_RX_BUF_SIZE;
}

// ===================== 旧接口（兼容） =====================
/**
 * @brief  获取一个蓝牙命令字节（旧接口）
 * @retval 返回接收到的字节；若无数据则返回 0
 * @note   不建议使用：可能将有效数据 0 误判为无命令，且无法处理多字节指令
 */
uint8_t Bluetooth_GetCmd(void)
{
    uint8_t data;
    if(Bluetooth_ReadByte(&data))
        return data;
    else
        return 0;
}
