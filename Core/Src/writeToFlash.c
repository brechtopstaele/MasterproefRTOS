#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <writeToFlash.h>
#include "eeprom.h"
//#include "main.h"

#include "cmsis_os.h"

uint16_t VirtAddVarTab[NB_OF_VAR];
uint16_t VarDataTab[NB_OF_VAR] = { 'M', 'a', 't', 'e', 'u', 's', 'z', ' ', 'S',
		'a', 'l', 'a', 'm', 'o', 'n', ' ', 'm', 's', 'a', 'l', 'a', 'm', 'o',
		'n', '.', 'p', 'l' };
uint8_t VarDataTabRead[NB_OF_VAR];
uint16_t VarDataTmp = 0;


/*void writeToFlash(UART_HandleTypeDef huart2, char data[100]){
	uint8_t dataLength = strlen(data);

	// Unlock the Flash Program Erase controller
	HAL_FLASH_Unlock();
	// Turn LED on
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	// EEPROM Init
	if (EE_Init() != EE_OK) {
		Error_Handler();
	}

	// Fill EEPROM variables addresses
	for (uint16_t i = 1; i <= dataLength; i++) {
		VirtAddVarTab[i - 1] = i;
	}

	// Store Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);
	for (uint16_t i = 0; i < dataLength; i++) {
		// Sequence 1
		if ((EE_WriteVariable(VirtAddVarTab[i], data[i]))
				!= HAL_OK) {
			Error_Handler();
		}
	}

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	for (uint16_t i = 0; i < dataLength; i++) {
		if ((EE_ReadVariable(VirtAddVarTab[i],
				&VarDataTabRead[i])) != HAL_OK) {
			Error_Handler();
		}
	}

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, dataLength, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
}*/

void writeToFlash(UART_HandleTypeDef huart2, char data[100]){
	uint8_t dataLength = strlen(data);

	// Unlock the Flash Program Erase controller
	HAL_FLASH_Unlock();

	/* EEPROM Init */
	if (EE_Init() != EE_OK) {
		Error_Handler();
	}

	// Fill EEPROM variables addresses
	for (uint16_t i = 1; i <= dataLength; i++) {
		VirtAddVarTab[i - 1] = i;
	}

	// Store Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);
	for (uint16_t i = 0; i < dataLength; i++) {
		/* Sequence 1 */
		if ((EE_WriteVariable(VirtAddVarTab[i], data[i]))
				!= HAL_OK) {
			Error_Handler();
		}
	}

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	for (uint16_t i = 0; i < dataLength; i++) {
		if ((EE_ReadVariable(VirtAddVarTab[i],
				&VarDataTabRead[i])) != HAL_OK) {
			Error_Handler();
		}
	}

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, dataLength, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
}

void readFlash(UART_HandleTypeDef huart2, char readData[100], uint8_t dataLength){
	//uint8_t readData[NB_OF_VAR];

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	for (uint16_t i = 0; i < dataLength; i++) {
		if ((EE_ReadVariable(VirtAddVarTab[i],
				&readData[i])) != HAL_OK) {
			Error_Handler();
		}
	}

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, readData, dataLength, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
}
