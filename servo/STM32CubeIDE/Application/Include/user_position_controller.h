/*
 * user_position_controller.h
 *
 *  Created on: May 14, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_POSITION_CONTROLLER_H_
#define APPLICATION_INCLUDE_USER_POSITION_CONTROLLER_H_

#include "pid_regulator.h"

typedef struct {
  int32_t inertia;
  int32_t bandwidthInertia;
  int32_t bandwidth;
  int32_t bandwidthMantissa;
} positionControllerConfig_t;

typedef struct {
  int32_t inertia;
  int32_t inertiaMantissa;
  int32_t kp;
  int32_t kpMantissa;
  int32_t ki;
  int32_t kiMantissa;
} positionInputFilter_t;

typedef struct {
  int32_t positionSetpoint;
  int32_t velocitySetpoint;
  int32_t torqueSetpoint;
  positionInputFilter_t inputPosFilter;
  PID_Handle_t posControl;
  PID_Handle_t velControl;
} positionController_t;


void POSCTRL_init();

void POSCTRL_update();

#endif /* APPLICATION_INCLUDE_USER_POSITION_CONTROLLER_H_ */
