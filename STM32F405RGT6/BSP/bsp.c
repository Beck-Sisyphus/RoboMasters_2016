#include "bsp.h"

#include "global_define.h"

void BSP_Init(void){

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    LED_Configuration();
    RELAY_Configuration();
    LASER_Configuration();
    CAN1_Configuration();
    CAN2_Configuration();
    USART1_Configuration();
    USART3_Configuration();
    PWM_Configuration();
    TIM2_Configuration();
    TIM6_Configuration();
}
