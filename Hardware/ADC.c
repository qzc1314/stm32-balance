#include "stm32f10x.h"                  // Device header

#include "adc.h"

#define ADC_RESOLUTION    4096.0f
#define VREF_VOLTAGE      3.3f
#define VOLTAGE_DIV_RATIO 5.0f

void Adc_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    /* STM32F103 的 ADC 时钟上限 14MHz。复位后 ADCPRE=00 时为 PCLK2/2=36MHz，
     * 超出规格会导致采样值偏小且不稳定，这里显式改为 72/6=12MHz。 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

static uint16_t Get_Adc_Single(void)
{
    uint32_t timeout = 100000;
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
        if(--timeout == 0) return 0;   // 超时返回 0
    return ADC_GetConversionValue(ADC1);
}

static uint16_t Get_Adc_Average(uint8_t times)
{
    uint32_t sum = 0;
    for(uint8_t i=0; i<times; i++)
        sum += Get_Adc_Single();
    return (uint16_t)(sum / times);
}

float Get_Battery_Voltage(void)
{
    uint16_t adc_val = Get_Adc_Average(10);
    return (float)adc_val / ADC_RESOLUTION * VREF_VOLTAGE * VOLTAGE_DIV_RATIO;
}
