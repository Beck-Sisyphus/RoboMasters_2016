#include "main.h"

#define GAP 0.0

/********************************************************************************
   ������巢��ָ�ID��Ϊ0x200��ֻ����������壬���ݻش�IDΪ0x201��0x202
	 cyq:����Ϊ�������������ָ�
*********************************************************************************/
void Cmd_ESC(int16_t current_201,int16_t current_202,int16_t current_203)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(current_201 >> 8);
    tx_message.Data[1] = (unsigned char)current_201;
    tx_message.Data[2] = (unsigned char)(current_202 >> 8);
    tx_message.Data[3] = (unsigned char)current_202;
    tx_message.Data[4] = (unsigned char)(current_203 >> 8);
    tx_message.Data[5] = (unsigned char)current_203;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}

/********************************************************************************
                         pitch��������ٶȻ�����
                    ���� pitch�ᵱǰ�ٶ� pitch��Ŀ���ٶ�
*********************************************************************************/
float Velocity_Control_201(float current_velocity_201,float target_velocity_201)
{
    const float v_p = 50.0;
    const float v_d = 0;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    
    if(abs(current_velocity_201) < GAP)
    {
        current_velocity_201 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_201 - current_velocity_201;
    
    output = error_v[1] * v_p             
             + (error_v[1] - error_v[0]) * v_d;
     
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;//cyq:for6015 ����
}


/********************************************************************************
                         pitch�������λ�û����� 
                    ���� pitch�ᵱǰλ�� pitch��Ŀ��λ��
*********************************************************************************/
float Position_Control_201(float current_position_201,float target_position_201)
{
    
    const float l_p = 20.5;
    const float l_i = 0.0;
    const float l_d = 0.0;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_201 - current_position_201;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return output;
}
/********************************************************************************
                           yaw��������ٶȻ�����
                      ���� yaw�ᵱǰ�ٶ� yaw��Ŀ���ٶ�
*********************************************************************************/
float Velocity_Control_203(float current_velocity_203,float target_velocity_203)
{
    const float v_p = 50.0;
    const float v_d = 0.0;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
		
    if(abs(current_velocity_203) < GAP)
    {
        current_velocity_203 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_203 - current_velocity_203;
    
    output = error_v[1] * v_p
             + (error_v[1] - error_v[0]) * v_d;
     
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;//cyq:for6015 ����
}

/********************************************************************************
                           yaw�������λ�û�����
                      ���� yaw�ᵱǰλ�� yaw��Ŀ��λ��
*********************************************************************************/
float Position_Control_203(float current_position_203,float target_position_203)
{
    const float l_p = 30.010;//3#5#:0.760
	const float l_i = 0.0;//0.000035;
    const float l_d = 0.5;//3.5;
    
    static float error_l[3] = {0.0,0.0,0.0};
    static float output = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = error_l[2];    
    error_l[2] = target_position_203 - current_position_203;
 
    output = error_l[2] * l_p 
							+ error_l[2] * l_i 
							+ (error_l[2] - error_l[1]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;
}
