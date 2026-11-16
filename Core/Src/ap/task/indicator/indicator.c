/*
 * indicator.c
 *
 *  Direction LEDs (front / left / right) and reverse-warning buzzer.
 *
 *  Pins are not configured in the .ioc, so they are initialized here.
 *  If you later add them in CubeMX, remove indicator_gpio_init() /
 *  buzzer_timer_init() and use the generated handles instead.
 */
#include "indicator.h"
#include "../../../driver/led/led.h"
#include "../../../driver/buzzer/buzzer.h"
#include <stdbool.h>

#define LED_FRONT_GPIO	GPIOB
#define LED_FRONT_PIN	GPIO_PIN_5	// D4
#define LED_LEFT_GPIO	GPIOB
#define LED_LEFT_PIN	GPIO_PIN_10	// D6
#define LED_RIGHT_GPIO	GPIOB
#define LED_RIGHT_PIN	GPIO_PIN_4	// D5

#define BUZZER_GPIO		GPIOA
#define BUZZER_PIN		GPIO_PIN_0	// A0, TIM2_CH1
#define BUZZER_CH		TIM_CHANNEL_1

#define BEEP_FREQ_HZ	1000
#define BEEP_PERIOD_MS	500		// 0.5s on / 0.5s off

// separate timer from TIM3 (motor): buzzer_on() changes ARR
TIM_HandleTypeDef htim2;

static led_t led_front, led_left, led_right;
static motor_state_t cur_motion = STOP;
static bool beep_on = false;
static uint32_t beep_tick = 0;

static void indicator_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOB, LED_FRONT_PIN | LED_LEFT_PIN | LED_RIGHT_PIN,
			GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = LED_FRONT_PIN | LED_LEFT_PIN | LED_RIGHT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void buzzer_timer_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = BUZZER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(BUZZER_GPIO, &GPIO_InitStruct);

	// 100MHz / 100 = 1MHz tick (buzzer driver assumes 1MHz: arr = 1e6/f - 1)
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 100 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		return;
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, BUZZER_CH);
}

static void indicator_apply_leds(motor_state_t motion) {
	led_off(&led_front);
	led_off(&led_left);
	led_off(&led_right);

	switch (motion) {
	case FORWARD:
		led_on(&led_front);
		break;
	case LEFT:
		led_on(&led_left);
		break;
	case RIGHT:
		led_on(&led_right);
		break;
	default: // STOP, BACKWARD: all off
		break;
	}
}

void indicator_init(void) {
	indicator_gpio_init();
	buzzer_timer_init();

	led_init(&led_front, LED_FRONT_GPIO, LED_FRONT_PIN);
	led_init(&led_left, LED_LEFT_GPIO, LED_LEFT_PIN);
	led_init(&led_right, LED_RIGHT_GPIO, LED_RIGHT_PIN);
	buzzer_init(&htim2, BUZZER_CH);

	cur_motion = STOP;
	beep_on = false;
	indicator_apply_leds(STOP);
	buzzer_off();
}

void indicator_update(motor_state_t motion) {
	uint32_t now = HAL_GetTick();

	// act only on state change (buzzer_on() rewrites ARR, don't spam it)
	if (motion != cur_motion) {
		cur_motion = motion;
		indicator_apply_leds(motion);

		if (motion == BACKWARD) {
			buzzer_on(BEEP_FREQ_HZ);
			beep_on = true;
			beep_tick = now;
		} else {
			buzzer_off();
			beep_on = false;
		}
	}

	// reverse warning: beep on/off every BEEP_PERIOD_MS
	if ((cur_motion == BACKWARD) && (now - beep_tick >= BEEP_PERIOD_MS)) {
		beep_tick = now;
		beep_on = !beep_on;
		if (beep_on) {
			buzzer_on(BEEP_FREQ_HZ);
		} else {
			buzzer_off();
		}
	}
}
