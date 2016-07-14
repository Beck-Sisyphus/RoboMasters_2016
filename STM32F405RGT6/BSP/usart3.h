#ifndef __USART3_H__
#define __USART3_H__

#include <stm32f4xx.h>
#include <stdio.h>

typedef struct {
    struct { 
        int16_t header;
        int8_t feeder_motor_state;
        int8_t friction_motor_state;
        int16_t pitch_req;
        int16_t yaw_req;
        int16_t feeder_motor_pwm;
        int16_t friction_motor_pwm;
        int16_t drive_req;
        int16_t strafe_req;
        int16_t rotate_req;
        int16_t mpu_x;
        int16_t mpu_y;
        int16_t mpu_z;
    } packet;
} arduino_data;

/**** forward declarations ****/
void USART3_Configuration(void);
void USART3_SendChar(unsigned char);
void RS232_Print( USART_TypeDef*, u8* );
void RS232_VisualScope_USART3( USART_TypeDef*, u8*, u16 );
#endif
