#include "global_define.h"

int g_module_id;

enum SYSTEM_STATE ge_system_state;//ϵͳ״̬

int g_hp = 1000;//Ѫ��

CanRxMsg g_CAN_Rx_message[CAN_RX_BUFFER_SIZE];

volatile unsigned char g_CAN_Rx_Provider;//can buffer ������
volatile unsigned char g_CAN_Rx_Consumer;//can buffer ������




