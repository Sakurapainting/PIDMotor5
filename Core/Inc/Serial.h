#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stdint.h"

extern uint8_t Serial_RxFlag;
extern uint8_t ByteRecv;

uint8_t Serial_GetRxFlag(void);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void hSerialSendByte(uint8_t byte);
void hSerialSendArray(uint8_t *array, uint16_t len);
void hSerialSendString(char *str);
void hSerial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_printf(char *format, ...);


#endif
