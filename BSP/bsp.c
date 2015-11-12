#include "bsp.h"

#include "global_define.h"

void BSP_Init(void){
    
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   
    LED_Configuration();            //ָʾ�Ƴ�ʼ��
    RELAY_Configuration();	    	//cyq:�̵�����ʼ��
    LASER_Configuration();
    CAN1_Configuration();            //��ʼ��CAN
    CAN2_Configuration(); 
    USART1_Configuration();          //����1��ʼ��
}


