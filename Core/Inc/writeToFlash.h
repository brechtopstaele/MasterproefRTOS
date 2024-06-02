// File for overwritable functions
#include "stm32f4xx_hal.h"

void writeToFlash(UART_HandleTypeDef huart2, char data[100]);
void readFlash(UART_HandleTypeDef huart2, char readData[100], uint8_t dataLength);
