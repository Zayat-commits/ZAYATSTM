/*
 * Blocks.h
 *
 *  Created on: May 20, 2020
 *      Author: abdul
 */

#ifndef INC_BLOCKS_H_
#define INC_BLOCKS_H_
#include "STD_TYPES.h"
#include "Constants.h"
#include "main.h"
#define PRINT_FLOAT_NO_TAB		0
#define PRINT_FLOAT_WITH_TAB	1
#define PRINT_INT_WITH_TAB		2
#define PRINT_INT_NO_TAB		3
#define PRINT_NORMAL			4

void vdBodyRatesBlock(parameters* ptr);
void vdDroneStartBlock(parameters* ptr);
void vdMPUBlock(parameters* ptr);
void vdPrintBlock(parameters* ptr);
void vdInsertBlock(parameters* ptr);
void vdOutputBlock(parameters* ptr);
void vdRollPitchBlock(parameters* ptr);
void vdYawBlock(parameters* ptr);
void vdAltitudeBlock(parameters* ptr);
void vdLateralBlock(parameters* ptr);

extern UART_HandleTypeDef huart1;
void fview(uint8_t type, float argument, char * line);
void string_receive(u8* buffer);
void vInitPARAMETERS(parameters *ptr);

#endif /* INC_BLOCKS_H_ */
