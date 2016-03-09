#include "main.h"

#define ADC1_DR_Address    ((u32)0x4001244C)
unsigned short ADC_ConvertedValue[256];

void ADC_Configuration(void)
{
    GPIO_InitTypeDef         gpio;
    ADC_InitTypeDef          adc;
    DMA_InitTypeDef          dma;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE); 
  
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&gpio);
    
    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    dma.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = 256;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    adc.ADC_Mode = ADC_Mode_Independent;	             
    adc.ADC_ScanConvMode = DISABLE;			             
    adc.ADC_ContinuousConvMode = ENABLE;	               
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);
  	ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

int ADC_Filter(void)
{
    int sum = 0;
    int adc_i = 0;
    
    for(adc_i=0;adc_i<256;adc_i++)
    { 
        sum += ADC_ConvertedValue[adc_i];
    }
    sum >>=8;
    
    return sum;
}
