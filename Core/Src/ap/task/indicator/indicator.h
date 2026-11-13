/*
 * indicator.h
 *
 *  Direction LEDs (front / left / right) and reverse-warning buzzer.
 */

#ifndef SRC_AP_TASK_INDICATOR_INDICATOR_H_
#define SRC_AP_TASK_INDICATOR_INDICATOR_H_

#include "../../../driver/dc_motor/dc_motor.h"

void indicator_init(void);
// call every loop with the current motion; handles LEDs and beep timing
void indicator_update(motor_state_t motion);

#endif /* SRC_AP_TASK_INDICATOR_INDICATOR_H_ */
