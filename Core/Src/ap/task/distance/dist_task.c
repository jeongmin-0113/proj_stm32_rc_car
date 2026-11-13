/*
 * dist_task.c
 *
 *  Created on: 2026. 11. 13.
 *      Author: kccistc
 */
#include "dist_task.h"
#include "../../../driver/HCSR04/HCSR04.h"
#include "cmsis_os.h"

#define SR04_FRONT_TRIG_GPIO	GPIOB
#define SR04_FRONT_TRIG_PIN		GPIO_PIN_2
#define SR04_FRONT_ECHO_GPIO	GPIOA
#define SR04_FRONT_ECHO_PIN		GPIO_PIN_9
#define SR04_LEFT_TRIG_GPIO		GPIOB
#define SR04_LEFT_TRIG_PIN		GPIO_PIN_1
#define SR04_LEFT_ECHO_GPIO		GPIOA
#define SR04_LEFT_ECHO_PIN		GPIO_PIN_8
#define SR04_RIGHT_TRIG_GPIO	GPIOC
#define SR04_RIGHT_TRIG_PIN		GPIO_PIN_4
#define SR04_RIGHT_ECHO_GPIO	GPIOA
#define SR04_RIGHT_ECHO_PIN		GPIO_PIN_10

sr04_sensor_t sr04_front, sr04_left, sr04_right;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case SR04_FRONT_ECHO_PIN:
		HCSR04_echoISR(&sr04_front);
		break;   // PA8
	case SR04_LEFT_ECHO_PIN:
		HCSR04_echoISR(&sr04_left);
		break;   // PA9
	case SR04_RIGHT_ECHO_PIN:
		HCSR04_echoISR(&sr04_right);
		break;   // PA10
	default:
		break;
	}
}

void dist_task_init() {
	HCSR04_init(&sr04_front, SR04_FRONT_TRIG_GPIO, SR04_FRONT_TRIG_PIN,
	SR04_FRONT_ECHO_GPIO, SR04_FRONT_ECHO_PIN);
	HCSR04_init(&sr04_left, SR04_LEFT_TRIG_GPIO, SR04_LEFT_TRIG_PIN,
	SR04_LEFT_ECHO_GPIO, SR04_LEFT_ECHO_PIN);
	HCSR04_init(&sr04_right, SR04_RIGHT_TRIG_GPIO, SR04_RIGHT_TRIG_PIN,
	SR04_RIGHT_ECHO_GPIO, SR04_RIGHT_ECHO_PIN);
}

void dist_measure(sr04_sensor_t *s, uint32_t *cm) {
	uint32_t us = 0;
	hcsr04_status_t status = HCSR04_start(s);

	if (status != HCSR04_OK) {
		*cm = DIST_INVALID;
		return;
	}
	do {
		osDelay(1);
		status = HCSR04_getResult(s, &us);
	} while (status == HCSR04_IN_PROGRESS);

	if (status == HCSR04_OK)
		*cm = us / 58;
	else
		*cm = DIST_INVALID;
}

void dist_task(uint32_t *front, uint32_t *left, uint32_t *right) {
	dist_measure(&sr04_front, front);
	osDelay(20);
	dist_measure(&sr04_left, left);
	osDelay(20);
	dist_measure(&sr04_right, right);
	osDelay(20);
}
