#include "main.h"

//�Ľ����򣺼��ٶ��˲�������Ԫ�������Ӽ��ٶȵ�У������
//ң����ָ��ƽ������

char id[3];
unsigned char USART_BUF[24] = {0};

int main(void)
{
    int i = 0;
    BSP_Init();
    //delay 500ms�� �ȴ�mpu6050�ϵ��ȶ�    
    delay_ms(500);    
    while(MPU6050_Initialization() == 0xff) 
    {
        i++;     //���һ�γ�ʼ��û�гɹ����Ǿ�����һ��                     
        if(i>10) //�����ʼ��һֱ���ɹ����Ǿ�ûϣ���ˣ�������ѭ����������һֱ��
        {
            while(1) 
            {
                LED1_TOGGLE();
                delay_ms(50);
                
            }
        }  
    }    
    MPU6050_Gyro_calibration();
    
    MPU6050_Interrupt_Configuration(); 
        
    PWM_Configuration();        
    //�趨ռ�ձ�Ϊ1000����ʼ��Ħ���ֵ��
    PWM1 = 1000;
    PWM2 = 1000;    
    delay_ms(1000);
    
    //��ʼ���͵��������
    Motor_Reset(MOTOR_NUM1);    
    delay_ms(30);//��ʱ        
    Motor_Init(MOTOR_NUM1,PWM_MODE);
    delay_ms(30);	
    
    while(1)
    {
        CurrentProtect();//�����������     
    }
}


