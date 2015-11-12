#include "main.h"

//改进方向：加速度滤波，在四元数中增加加速度的校正比重
//遥控器指令平滑处理

char id[3];
unsigned char USART_BUF[24] = {0};

int main(void)
{
    int i = 0;
    BSP_Init();
    //delay 500ms， 等待mpu6050上电稳定    
    delay_ms(500);    
    while(MPU6050_Initialization() == 0xff) 
    {
        i++;     //如果一次初始化没有成功，那就再来一次                     
        if(i>10) //如果初始化一直不成功，那就没希望了，进入死循环，蜂鸣器一直叫
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
    //设定占空比为1000，初始化摩擦轮电调
    PWM1 = 1000;
    PWM2 = 1000;    
    delay_ms(1000);
    
    //初始化送弹电机驱动
    Motor_Reset(MOTOR_NUM1);    
    delay_ms(30);//延时        
    Motor_Init(MOTOR_NUM1,PWM_MODE);
    delay_ms(30);	
    
    while(1)
    {
        CurrentProtect();//电调电流保护     
    }
}


