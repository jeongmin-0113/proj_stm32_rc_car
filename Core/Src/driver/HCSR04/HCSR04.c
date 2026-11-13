/*
 * HCSR04.c
 *
 *  Created on: 2026. 10. 28.
 *      Author: rhoblack
 */

#include "HCSR04.h"
#include "../../common/delay/delay.h"
#include <stdbool.h>

#define RISE_TIMEOUT_MS		5
#define FALL_TIMEOUT_MS		50
#define MAX_ECHO_US       25000

void HCSR04_init(sr04_sensor_t *s, GPIO_TypeDef *t_port, uint16_t t_pin,
		GPIO_TypeDef *e_port, uint16_t e_pin) {
	s->trig_port = t_port;
	s->trig_pin = t_pin;
	s->echo_port = e_port;
	s->echo_pin = e_pin;

	delay_init();
	HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
	s->state = STATE_IDLE;
}

static bool echo_is_high(sr04_sensor_t *s) {
	return (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET);
}

hcsr04_status_t HCSR04_start(sr04_sensor_t *s) {
	if (s->state == STATE_WAIT_RISE || s->state == STATE_WAIT_FALL) {
		return HCSR04_ERR_BUSY;
	}
	if (echo_is_high(s)) {
		return HCSR04_ERR_BUSY;
	}
	s->state = STATE_WAIT_RISE;
	s->start_ms = HAL_GetTick();

	HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

	return HCSR04_OK;
}

void HCSR04_echoISR(sr04_sensor_t *s) {
	uint32_t cnt = DWT->CYCCNT;
	bool high = echo_is_high(s);

	if ((s->state == STATE_WAIT_RISE) && high) {
		s->rise_cnt = cnt;
		s->state = STATE_WAIT_FALL;
	} else if ((s->state == STATE_WAIT_FALL) && !high) {
		s->fall_cnt = cnt;
		s->state = STATE_DONE;
	}
}

hcsr04_status_t HCSR04_getResult(sr04_sensor_t *s, uint32_t *echo_us) {
	uint32_t elapsed_ms = HAL_GetTick() - s->start_ms;

	switch (s->state) {
	case STATE_WAIT_RISE:
		if (elapsed_ms <= RISE_TIMEOUT_MS) {
			return HCSR04_IN_PROGRESS;
		}
		s->state = STATE_IDLE;
		return HCSR04_ERR_NO_ECHO;
	case STATE_WAIT_FALL:
		if (elapsed_ms <= FALL_TIMEOUT_MS) {
			return HCSR04_IN_PROGRESS;
		}
		s->state = STATE_IDLE;
		return HCSR04_ERR_NO_ECHO;
	case STATE_DONE: {
		uint32_t us = (s->fall_cnt - s->rise_cnt) / (SystemCoreClock / 1000000);
		s->state = STATE_IDLE;
		*echo_us = us;
		if (us > MAX_ECHO_US)
			return HCSR04_ERR_OUT_OF_RANGE;
		return HCSR04_OK;
	}
	case STATE_IDLE:
	default:
		return HCSR04_ERR_NOT_STARTED;
	}
}

