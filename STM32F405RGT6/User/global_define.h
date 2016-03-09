#ifndef __GLOBAL_DEFINE_H__
#define __GLOBAL_DEFINE_H__

//CAN Bus Base address
#define CAN_BASE_ADDR 84
//CAN Bus maximum module
#define CAN_ADDR_MAX 7 


#define MODULE_ID     8
#define CAN_RX_BUFFER_SIZE 20

#include <stm32f4xx.h>

extern int g_module_id;

enum SYSTEM_STATE{
    PLAYING,    // in game
    IAP         // perform IAP programming for weak points
};

extern enum SYSTEM_STATE ge_system_state;
extern int g_hp;// Blood

extern CanRxMsg g_CAN_Rx_message[CAN_RX_BUFFER_SIZE];

extern volatile unsigned char g_CAN_Rx_Provider;//can buffer Provider
extern volatile unsigned char g_CAN_Rx_Consumer;//can buffer Consumer


#endif 
