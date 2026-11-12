/*
 * bt_task.c
 *
 *  Created on: 2026. 11. 12.
 *      Author: kccistc
 */
#include "bt_task.h"
#include "../../message.h"
#include "../../../driver/lcd/lcd.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern UART_HandleTypeDef huart6;

static uint8_t rx_data = 0;
static uint8_t rx_done = 0;

static void bt_task_rx_cplt(UART_HandleTypeDef *huart) {
	rx_done = 1;
	HAL_UART_Receive_IT(&huart6, &rx_data, 1);
}

void bt_task_init() {
	rx_data = 0;
	rx_done = 0;
	HAL_UART_RegisterCallback(&huart6, HAL_UART_RX_COMPLETE_CB_ID,
			bt_task_rx_cplt);
	HAL_UART_Receive_IT(&huart6, &rx_data, 1);
	printf("\r\n=== RC CAR BOOT ===\r\n");
}

void bt_task() {
	if (rx_done) {
		if (rx_data == 'S') {
			printf("S\n");
			osMessagePut(BtQueueHandle, BT_STOP, 10);
			if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
				Lcd_writeStringXY(1, 0, "BT Task STOP      ");
				osMutexRelease(lcdMutexHandle);
			}
		} else if (rx_data == 'F') {
			printf("F\n");
			osMessagePut(BtQueueHandle, BT_FRONT, 10);
			if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
				Lcd_writeStringXY(1, 0, "BT Task FRONT      ");
				osMutexRelease(lcdMutexHandle);
			}
		} else if (rx_data == 'B') {
			printf("B\n");
			osMessagePut(BtQueueHandle, BT_BACK, 10);
			if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
				Lcd_writeStringXY(1, 0, "BT Task BACK      ");
				osMutexRelease(lcdMutexHandle);
			}
		} else if (rx_data == 'L') {
			printf("L\n");
			osMessagePut(BtQueueHandle, BT_LEFT, 10);
			if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
				Lcd_writeStringXY(1, 0, "BT Task LEFT      ");
				osMutexRelease(lcdMutexHandle);
			}
		} else if (rx_data == 'R') {
			printf("R\n");
			osMessagePut(BtQueueHandle, BT_RIGHT, 10);
			if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
				Lcd_writeStringXY(1, 0, "BT Task RIGHT      ");
				osMutexRelease(lcdMutexHandle);
			}
		} else {
			osMessagePut(BtQueueHandle, BT_NO, 10);
		}
		rx_done = 0;
	}
}
