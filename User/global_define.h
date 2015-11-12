#ifndef __GLOBAL_DEFINE_H__
#define __GLOBAL_DEFINE_H__

//CAN设备基址
#define CAN_BASE_ADDR 84
//挂载模块个数上限
#define CAN_ADDR_MAX 7 


#define MODULE_ID     8
#define CAN_RX_BUFFER_SIZE 20

#include <stm32f4xx.h>

extern int g_module_id;

enum SYSTEM_STATE{
    PLAYING,    //游戏进行中
    IAP         //对薄弱点进行iap编程
};

extern enum SYSTEM_STATE ge_system_state;
extern int g_hp;//血量

extern CanRxMsg g_CAN_Rx_message[CAN_RX_BUFFER_SIZE];

extern volatile unsigned char g_CAN_Rx_Provider;//can buffer 生产者
extern volatile unsigned char g_CAN_Rx_Consumer;//can buffer 消费者


#endif 
