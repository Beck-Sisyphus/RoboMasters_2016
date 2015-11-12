#include "bsp.h"

#include "global_define.h"

void BSP_Init(void){
    
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   
    LED_Configuration();            //指示灯初始化
    RELAY_Configuration();	    	//cyq:继电器初始化
    LASER_Configuration();
    CAN1_Configuration();            //初始化CAN
    CAN2_Configuration(); 
    USART1_Configuration();          //串口1初始化
}


