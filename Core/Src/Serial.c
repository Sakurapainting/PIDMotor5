#include "usart.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>


uint8_t Serial_RxFlag;
uint8_t ByteRecv;

uint8_t Serial_GetRxFlag(void){
  if(Serial_RxFlag == 1){
    Serial_RxFlag = 0;
    return 1;
  }
  return 0;
}

//串口发送字节
void hSerialSendByte(uint8_t byte){
  HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY); 

}

//串口发送数组
void hSerialSendArray(uint8_t *array, uint16_t len){
  HAL_UART_Transmit(&huart1, array, len, HAL_MAX_DELAY); 
}

//串口发送字符串
void hSerialSendString(char *str){
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY); 
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y){
    uint32_t result = 1;
    while(Y--){
        result *= X;
    }
    return result;
}

//串口发送数字
void hSerial_SendNumber(uint32_t Number, uint8_t Length){
    uint8_t i;
    for(i = 0; i < Length; i++){
        hSerialSendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');   //'0'根据ASCII码表将数字转换为字符
    }
}

//串口printf
void Serial_printf(char *format, ...){
    char String[100];
    va_list args;
    va_start(args, format);
    vsprintf(String, format, args);
    va_end(args);
    hSerialSendString(String);
}
