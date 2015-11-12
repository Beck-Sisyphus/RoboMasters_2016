#include "global_define.h"

int g_module_id;

enum SYSTEM_STATE ge_system_state;//系统状态

int g_hp = 1000;//血量

CanRxMsg g_CAN_Rx_message[CAN_RX_BUFFER_SIZE];

volatile unsigned char g_CAN_Rx_Provider;//can buffer 生产者
volatile unsigned char g_CAN_Rx_Consumer;//can buffer 消费者




