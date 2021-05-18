/*
 * user_servo_controller.h
 *
 *  Created on: May 18, 2021
 *      Author: sunbl
 */

#ifndef APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_
#define APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_

#include "stdbool.h"
#include "stdint.h"

typedef struct {
  float IndexScanSpeed;
  float StepAngle;
  float VelMaxAbs;
  float TorMaxAbs;
  float InputFiltKp;
  float InputFiltKi;
  float Inertia;
} ServoConfig_t;

typedef enum {
  DISABLED,
  ALIGNING,
  ENABLED
} ServoState_t ;

typedef struct {

  ServoConfig_t Config;

  ServoState_t State;

  float PosSetpoint, VelSetpoint, TorSetpoint;

  int32_t EncoderOffset, StepDirOffset;

  ENCODER_Handle_t *Encoder;
  SpeednTorqCtrl_Handle_t *TorqueController;
  PID_Handle_t *PIDPosRegulator, *PIDVelRegulator;
} Servo_t;


void SERVO_Init(PosCtrl_Handle_t * self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t * TorqueController, ServoConfig_t Config);
void SERVO_ControlPosition(PosCtrl_Handle_t * self);
void SERVO_SetEncoderOffset(PosCtrl_Handle_t * self);
void SERVO_Disable(PosCtrl_Handle_t * self);
void SERVO_Align(PosCtrl_Handle_t * self);
void SERVO_Enable(PosCtrl_Handle_t * self);
bool SERVO_IsAlignmentComplete(PosCtrl_Handle_t * self);

#endif /* APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_ */
