/*
 * user_position_controller.c
 *
 *  Created on: May 14, 2021
 *      Author: Sam Blazes
 */



#include "user_position_controller.h"
#include "mc_api.h"



void POSCTRL_init(positionController_t *handle, positionControllerConfig_t config) {

  handle->inputPosFilter.ki = 2 * config.bandwidth;
}

void POSCTRL_update(positionController_t *handle, int32_t dt, int32_t dtMantissa, int32_t inputPosition) {

  // calculate filtered position setpoint using a second order filter
  // int32_t deltaPos = inputPosition - handle->posSetpoint;
  // int32_t deltaVel = 0 - handle->velSetpoint;

  // int32_t propTerm = handle->inputPosFilter.kp * deltaPos;
  // int32_t intTerm = handle->inputPosFilter.ki * deltaVel;
  // int32_t accel = (propTerm >> handle->inputPosFilter.kpMantissa) + (intTerm >> handle->inputPosFilter.kiMantissa);

  // handle->torqueSetpoint = (accel * handle->inputPosFilter.inertia) >> handle->inputPosFilter.inertiaMantissa;
  // handle->velocitySetpoint = (dt * accel) >> dtMantissa;
  // handle->positionSetpoint = (dt * handle->velocitySetpoint) >> dtMantissa;


  // SpeednPosFdbk_Handle_t *speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  // int16_t position = SPD_GetMecAngle(speedHandle);
  // int16_t velocity = SPD_GetAvrgMecSpeedUnit(speedHandle);

}
