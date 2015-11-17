#ifndef __USART1_H__
#define __USART1_H__

#include <stm32f4xx.h>
#include <stdio.h>

void USART1_Configuration(void);
void USART1_SendChar(unsigned char b);
void RS232_Print( USART_TypeDef*, u8* );
void RS232_VisualScope( USART_TypeDef*, u8*, u16 );
#endif
