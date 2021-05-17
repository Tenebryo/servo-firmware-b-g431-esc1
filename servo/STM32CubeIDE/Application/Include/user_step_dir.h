/*
 * user_step_dir.h
 *
 * Contains functions to retrieve the position set point from the timer.
 *
 *  Created on: May 15, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_STEP_DIR_H_
#define APPLICATION_INCLUDE_USER_STEP_DIR_H_

#include "stdbool.h"
#include "stdint.h"

void STEPDIR_Init(void);
bool STEPDIR_ReadEnable(void);
uint32_t STEPDIR_GetInputPosition(void);


#endif /* APPLICATION_INCLUDE_USER_STEP_DIR_H_ */
