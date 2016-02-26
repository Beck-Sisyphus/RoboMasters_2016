#ifndef __USART3_H__
#define __USART3_H__

#include <stm32f4xx.h>
#include <stdio.h>

// /**** rc channel definition ****/
// #define RC_CH_VALUE_MIN                 ((uint16_t)364 )
// #define RC_CH_VALUE_OFFSET              ((uint16_t)1024)
// #define RC_CH_VALUE_MAX                 ((uint16_t)1684)

// /**** rc switch definition ****/
// #define RC_SW_UP                        ((uint16_t)1)
// #define RC_SW_MID                       ((uint16_t)3)
// #define RC_SW_DOWN                      ((uint16_t)2)

// /**** pc key definition ****/
// #define KEY_PRESSED_OFFSET_W            ((uint16_t)0x01<<0)
// #define KEY_PRESSED_OFFSET_S            ((uint16_t)0x01<<1)
// #define KEY_PRESSED_OFFSET_A            ((uint16_t)0x01<<2)
// #define KEY_PRESSED_OFFSET_D            ((uint16_t)0x01<<3)
// #define KEY_PRESSED_OFFSET_Q            ((uint16_t)0x01<<4)
// #define KEY_PRESSED_OFFSET_E            ((uint16_t)0x01<<5)
// #define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)0x01<<6)
// #define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)0x01<<7)

// /**** data struct ****/
// typedef struct {
//     struct { 
//         uint16_t ch0;
//         uint16_t ch1;
//         uint16_t ch2;
//         uint16_t ch3;
//         uint8_t s1;
//         uint8_t s2;
//     } rc;
//     struct {
//         int16_t x;
//         int16_t y;
//         int16_t z;
//         uint8_t press_l;
//         uint8_t press_r;
//     } mouse;
//     struct {
//         uint16_t v;
//     } key;
// } RC_Ctl_t;

/**** forward declarations ****/
void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
void RS232_Print( USART_TypeDef*, u8* );
void RS232_VisualScope( USART_TypeDef*, u8*, u16 );
#endif
