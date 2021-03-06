#include "main.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

void MPU6050_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);

	  gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_Init(GPIOA, &gpio);

    // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource4);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4);

    exti.EXTI_Line = EXTI_Line4;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0x0F; // Lowest prioity
    nvic.NVIC_IRQChannelSubPriority = 0x0F;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

//MPU6050 external interrupt handler
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) == SET)
    {
        // Read MPU6050 data out to control the gimbal smoother
        // USe MPU6050 gyro output as the close-loop feedback for speed
        // If only use the mechanical angle from motor controller for feedback
        // Significant oscillation occur
			  MPU6050_ReadData();
        EXTI_ClearFlag(EXTI_Line5);
        EXTI_ClearITPendingBit(EXTI_Line5);
			  // MPU6050_ReadData();
    }
}
