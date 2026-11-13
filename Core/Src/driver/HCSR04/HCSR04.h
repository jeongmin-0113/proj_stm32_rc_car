/*
 * HCSR04.h
 *
 *  Created on: 2026. 10. 28.
 *      Author: rhoblack
 */

#ifndef SRC_DRIVER_HCSR04_HCSR04_H_
#define SRC_DRIVER_HCSR04_HCSR04_H_

#include "stm32f4xx_hal.h"

typedef enum {
	HCSR04_OK = 0,
	HCSR04_IN_PROGRESS,
	HCSR04_ERR_NOT_STARTED,
	HCSR04_ERR_BUSY,
	HCSR04_ERR_NO_ECHO,
	HCSR04_ERR_OUT_OF_RANGE
} hcsr04_status_t;

typedef enum {
	STATE_IDLE,
	STATE_WAIT_RISE,
	STATE_WAIT_FALL,
	STATE_DONE
}sr04_state_t;

typedef struct {
	GPIO_TypeDef *trig_port;
	uint16_t trig_pin;
	GPIO_TypeDef *echo_port;
	uint16_t echo_pin;
	volatile sr04_state_t state;
	volatile uint32_t rise_cnt;
	volatile uint32_t fall_cnt;
	uint32_t start_ms;
} sr04_sensor_t;

void HCSR04_init(sr04_sensor_t* s, GPIO_TypeDef *t_port, uint16_t t_pin,
		GPIO_TypeDef *e_port, uint16_t e_pin);
hcsr04_status_t HCSR04_start(sr04_sensor_t* s);
void HCSR04_echoISR(sr04_sensor_t* s);
hcsr04_status_t HCSR04_getResult(sr04_sensor_t* s, uint32_t *echo_us);

#endif /* SRC_DRIVER_HCSR04_HCSR04_H_ */
