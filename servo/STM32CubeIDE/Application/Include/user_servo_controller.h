/*
 * user_servo_controller.h
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_
#define APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_

#include "mc_api.h"
#include "speed_pos_fdbk.h"
#include "encoder_speed_pos_fdbk.h"
#include "user_step_dir.h"
#include "user_fpid_regulator.h"

typedef struct {
  float IndexScanSpeed;
  float StepAngle;
  float VelMaxAbs;
  float TorMaxAbs;
  float InputFiltKp;
  float InputFiltKi;
  float Inertia;
  float TorqueBandwidth;
} ServoConfig_t;

typedef enum {
  UNINIT,
  DISABLED,
  ALIGNING,
  ENABLED
} ServoState_t ;

typedef struct {

  ServoConfig_t Config;

  ServoState_t State;

  float PosSetpoint, VelSetpoint, TorSetpoint;

  bool Aligned;

  int32_t EncoderOffset, StepDirOffset;

  ENCODER_Handle_t *Encoder;
  SpeednTorqCtrl_Handle_t *TorqueController;
  FPID_Handle_t *PIDPosRegulator, *PIDVelRegulator;
} Servo_t;


void SERVO_Init(Servo_t *self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t *TorqueController, FPID_Handle_t *PIDPosRegulator, FPID_Handle_t *PIDVelRegulator);
void SERVO_ControlPosition(Servo_t * self, float DeltaTime, float InputPos);
void SERVO_ControlPositionFromStepDir(Servo_t * self, float DeltaTime);
void SERVO_SetEncoderOffset(Servo_t * self);
void SERVO_Disable(Servo_t * self);
void SERVO_Align(Servo_t * self);
void SERVO_Enable(Servo_t * self);
bool SERVO_IsAlignmentComplete(Servo_t * self);

#endif /* APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_ */
