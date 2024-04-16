#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "eeprom.h"
//#include "main.h"

#include "overwrite.h"
#include "cmsis_os.h"

uint16_t VirtAddVarTab[NB_OF_VAR];
uint16_t VarDataTab[NB_OF_VAR] = { 'M', 'a', 't', 'e', 'u', 's', 'z', ' ', 'S',
		'a', 'l', 'a', 'm', 'o', 'n', ' ', 'm', 's', 'a', 'l', 'a', 'm', 'o',
		'n', '.', 'p', 'l' };
uint8_t VarDataTabRead[NB_OF_VAR];
uint16_t VarIndex, VarDataTmp = 0;


void writeToFlash(UART_HandleTypeDef huart2){
	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	/* EEPROM Init */
	if (EE_Init() != EE_OK) {
		Error_Handler();
	}

	// Fill EEPROM variables addresses
	for (VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++) {
		VirtAddVarTab[VarIndex - 1] = VarIndex;
	}

	// Store Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		/* Sequence 1 */
		if ((EE_WriteVariable(VirtAddVarTab[VarIndex], VarDataTab[VarIndex]))
				!= HAL_OK) {
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		if ((EE_ReadVariable(VirtAddVarTab[VarIndex],
				&VarDataTabRead[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);

	// Store revert Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
	HAL_UART_Transmit(&huart2, "Store revert values\n\r", 21, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		/* Sequence 1 */
		if ((EE_WriteVariable(VirtAddVarTab[VarIndex],
				VarDataTab[NB_OF_VAR - VarIndex - 1])) != HAL_OK) {
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	// Read values
	HAL_UART_Transmit(&huart2, "Read revert values\n\r", 20, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		if ((EE_ReadVariable(VirtAddVarTab[VarIndex],
				&VarDataTabRead[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, "Read revert table: ", 19, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);

	// Store Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
	HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		/* Sequence 1 */
		if ((EE_WriteVariable(VirtAddVarTab[VarIndex], VarDataTab[VarIndex]))
				!= HAL_OK) {
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		if ((EE_ReadVariable(VirtAddVarTab[VarIndex],
				&VarDataTabRead[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
}
