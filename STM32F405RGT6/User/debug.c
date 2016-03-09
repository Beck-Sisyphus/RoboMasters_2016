#include "main.h"

void CAN_Send_Debug_207(float data1,float data2)
{
    CanTxMsg tx_message;
    unsigned char *p_data1;
    unsigned char *p_data2;
    
    p_data1 = (unsigned char *)&data1;
    p_data2 = (unsigned char *)&data2;
    
    tx_message.StdId = 0x207;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = *p_data1++;
    tx_message.Data[1] = *p_data1++;
    tx_message.Data[2] = *p_data1++;
    tx_message.Data[3] = *p_data1;
    tx_message.Data[4] = *p_data2++;
    tx_message.Data[5] = *p_data2++;
    tx_message.Data[6] = *p_data2++;
    tx_message.Data[7] = *p_data2;
    
    CAN_Transmit(CAN1,&tx_message);
}


