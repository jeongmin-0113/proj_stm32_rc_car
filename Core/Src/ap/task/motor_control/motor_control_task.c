/*
 * motor_control_task.c
 *
 *  Created on: 2026. 11. 12.
 *      Author: kccistc
 */
#include "motor_control_task.h"
#include "../../../driver/dc_motor/dc_motor.h"
#include "../../message.h"
#include "../comm/comm_task.h"
#include "../../../driver/lcd/lcd.h"
#include <stdbool.h>

motor_t motorL, motorR;
motor_state_t motor_state = STOP;

extern TIM_HandleTypeDef htim3;
extern uint32_t front, left, right;

void motor_control_task_init(void) {
	motor_init(&motorL, &htim3, TIM_CHANNEL_1, MOTOR_LEFT1_GPIO,
	MOTOR_LEFT1_PIN, MOTOR_LEFT2_GPIO, MOTOR_LEFT2_PIN);
	motor_init(&motorR, &htim3, TIM_CHANNEL_3, MOTOR_RIGHT1_GPIO,
	MOTOR_RIGHT1_PIN, MOTOR_RIGHT2_GPIO, MOTOR_RIGHT2_PIN);
	motor_setSpeed(900);
}

bool motor_getCondition(btn_status_t btn_status, comm_status_t comm_status,
		bt_status_t bt_status, motor_state_t dir) {
	switch (dir) {
	case STOP:
		return (btn_status == BTN_RUN_STOP) || (comm_status == COMM_STOP)
				|| (bt_status == BT_STOP);
		break;
	case FORWARD:
		return (btn_status == BTN_RUN_STOP) || (comm_status == COMM_RUN)
				|| (bt_status == BT_FRONT);
		break;
	case BACKWARD:
		return (bt_status == BT_BACK);
		break;
	case RIGHT:
		return (btn_status == BTN_RIGHT) || (bt_status == BT_RIGHT);
		break;
	case LEFT:
		return (btn_status == BTN_LEFT) || (bt_status == BT_LEFT);
		break;
	default:
		return false;
		break;
	}
}

#define DIST_DANGER	20
#define DIST_SAFE	40
#define DIST_INVALID 0xFFFFFFFF

typedef enum {
	AUTO_STOP, AUTO_STRAIGHT, AUTO_LEFT, AUTO_RIGHT, AUTO_BACK
} auto_mobile_state_t;

bool motor_stucked() {
	return (front <= DIST_DANGER) && (left <= DIST_DANGER)
			&& (right <= DIST_DANGER);
}

void motor_auto_control_task(void) {
	static uint32_t d_front = 0, d_left = 0, d_right = 0;
	static auto_mobile_state_t state = AUTO_STOP;

	bt_status_t bt_status = BT_NO;
	osEvent bt_evt = osMessageGet(BtQueueHandle, 0);
	if (bt_evt.status == osEventMessage) {
		bt_status = bt_evt.value.v;
	}

	d_front = (front == DIST_INVALID) ? d_front : front;

	switch (state) {
	case AUTO_STOP:
		if ((bt_status == BT_FRONT)) {
			state = AUTO_STRAIGHT;
		}

		motor_stop(&motorL, &motorR);
		break;
	case AUTO_STRAIGHT:
		if ((bt_status == BT_STOP) || (motor_stucked())) {
			state = AUTO_STOP;
		} else if (d_front <= DIST_DANGER) {
			d_left = (left == DIST_INVALID) ? d_left : left;
			d_right = (right == DIST_INVALID) ? d_right : right;

			if (d_left >= d_right) {
				state = AUTO_LEFT;
			} else {
				state = AUTO_RIGHT;
			}
		}

		motor_forward(&motorL, &motorR);
		break;
	case AUTO_LEFT:
		if ((bt_status == BT_STOP) || (motor_stucked())) {
			state = AUTO_STOP;
		} else if (d_front >= DIST_SAFE) {
			state = AUTO_STRAIGHT;
		}

		motor_left(&motorL, &motorR);
		break;
	case AUTO_RIGHT:
		if ((bt_status == BT_STOP) || (motor_stucked())) {
			state = AUTO_STOP;
		} else if (d_front >= DIST_SAFE) {
			state = AUTO_STRAIGHT;
		}

		motor_right(&motorL, &motorR);
		break;
	default:
		state = AUTO_STOP;
		motor_stop();
		break;
	}
}

void motor_control_task(void) {
	osEvent btn_evt, comm_evt, bt_evt;
	btn_status_t btn_status = BTN_NO;
	btn_evt = osMessageGet(BtnQueueHandle, 0);
	if (btn_evt.status == osEventMessage) {
		btn_status = btn_evt.value.v;
	}

	comm_status_t comm_status = COMM_NO;
	int speed = 0;
	comm_evt = osMessageGet(CommQueueHandle, 0);
	if (comm_evt.status == osEventMessage) {
		comm_status = ((comm_data_t*) comm_evt.value.p)->status;
		speed = ((comm_data_t*) comm_evt.value.p)->speed;
	}

	bt_status_t bt_status = BT_NO;
	bt_evt = osMessageGet(BtQueueHandle, 0);
	if (bt_evt.status == osEventMessage) {
		bt_status = bt_evt.value.v;
	}

	if (comm_status == COMM_SPEED) {
		motor_setSpeed(speed);
	}

	switch (motor_state) {
	case STOP:
		if (motor_getCondition(btn_status, comm_status, bt_status, FORWARD)) {
			motor_state = FORWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				BACKWARD)) {
			motor_state = BACKWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				RIGHT)) {
			motor_state = RIGHT;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				LEFT)) {
			motor_state = LEFT;
		}
		motor_stop(&motorL, &motorR);

		if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
			Lcd_writeStringXY(0, 0, "Motor Task STOP   ");
			osMutexRelease(lcdMutexHandle);
		}
		break;
	case FORWARD:
		if (motor_getCondition(btn_status, comm_status, bt_status, STOP)) {
			motor_state = STOP;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				BACKWARD)) {
			motor_state = BACKWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				RIGHT)) {
			motor_state = RIGHT;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				LEFT)) {
			motor_state = LEFT;
		}

		motor_forward(&motorL, &motorR);

		if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
			Lcd_writeStringXY(0, 0, "Motor Task RUN   ");
			osMutexRelease(lcdMutexHandle);
		}
		break;
	case BACKWARD:
		if (motor_getCondition(btn_status, comm_status, bt_status, STOP)) {
			motor_state = STOP;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				FORWARD)) {
			motor_state = FORWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				RIGHT)) {
			motor_state = RIGHT;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				LEFT)) {
			motor_state = LEFT;
		}

		motor_backward(&motorL, &motorR);
		if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
			Lcd_writeStringXY(0, 0, "Motor Task BACK   ");
			osMutexRelease(lcdMutexHandle);
		}
		break;
	case RIGHT:
		if (motor_getCondition(btn_status, comm_status, bt_status, STOP)) {
			motor_state = STOP;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				FORWARD)) {
			motor_state = FORWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				BACKWARD)) {
			motor_state = BACKWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				LEFT)) {
			motor_state = LEFT;
		}

		motor_right(&motorL, &motorR);
		if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
			Lcd_writeStringXY(0, 0, "Motor Task RIGHT   ");
			osMutexRelease(lcdMutexHandle);
		}
		break;
	case LEFT:
		if (motor_getCondition(btn_status, comm_status, bt_status, STOP)) {
			motor_state = STOP;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				FORWARD)) {
			motor_state = FORWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				BACKWARD)) {
			motor_state = BACKWARD;
		} else if (motor_getCondition(btn_status, comm_status, bt_status,
				RIGHT)) {
			motor_state = RIGHT;
		}

		motor_left(&motorL, &motorR);

		if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK) {
			Lcd_writeStringXY(0, 0, "Motor Task LEFT   ");
			osMutexRelease(lcdMutexHandle);
		}
		break;
	default:
		motor_stop(&motorL, &motorR);
		break;
	}

}
