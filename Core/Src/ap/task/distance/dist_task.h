/*
 * dist_task.h
 *
 *  Created on: 2026. 11. 13.
 *      Author: kccistc
 */

#ifndef SRC_AP_TASK_DISTANCE_DIST_TASK_H_
#define SRC_AP_TASK_DISTANCE_DIST_TASK_H_

#include <stdint.h>

#define DIST_INVALID 0xFFFFFFFF

void dist_task_init();
void dist_task(uint32_t *front, uint32_t *left, uint32_t *right);

#endif /* SRC_AP_TASK_DISTANCE_DIST_TASK_H_ */
