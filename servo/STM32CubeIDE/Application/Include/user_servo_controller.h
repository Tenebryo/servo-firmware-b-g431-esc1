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

extern int16_t LastTorqueApplied;
extern float ServoTempInputPos;

// #define COGGING_TORQUE_POINTS 128
// #define COG_INDEX(step) (((step) * COGGING_TORQUE_POINTS) / (4 * M1_ENCODER_PPR))
// #define COG_STEP(step) (((step) * (4 * M1_ENCODER_PPR)) / COGGING_TORQUE_POINTS)

typedef struct {
  float IndexScanSpeed;
  float TurnsPerStep;
  float VelMaxAbs;
  float TorMaxAbs;
  float InputFiltKp;
  float InputFiltKi;
  float Inertia;
  float TorqueBandwidth;
  bool EncoderDirectionFlipped;
  // float CoggingTorque[COGGING_TORQUE_POINTS];
} ServoConfig_t;

typedef enum {
  UNINIT,
  DISABLED,
  ALIGNING,
  ENABLED_STEP_DIRECTION,
  ENABLED_POSITION_FILTER,
  ENABLED_PID,
  ENABLED_PIV,
} ServoState_t ;

typedef struct {

  ServoConfig_t Config;

  ServoState_t State;

  float PosInput, VelInput, TorInput;
  float PosSetpoint, VelSetpoint, TorSetpoint;

  bool Aligned;

  int32_t EncoderOffset, StepDirOffset;
  uint32_t LastEncoderCount;
  int32_t EncoderPosition;

  ENCODER_Handle_t *Encoder;
  SpeednTorqCtrl_Handle_t *TorqueController;
  FPID_Handle_t *PIDPosRegulator, *PIVPosRegulator, *PIVVelRegulator;
} Servo_t;


void SERVO_Init(Servo_t *self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t *TorqueController, FPID_Handle_t *PIDPosRegulator, FPID_Handle_t *PIVPosRegulator, FPID_Handle_t *PIVVelRegulator);
void SERVO_ControlPosition(Servo_t * self, float DeltaTime);
void SERVO_ResetEncoderOffset(Servo_t * self);
void SERVO_Disable(Servo_t * self);
void SERVO_Align(Servo_t * self);
void SERVO_EnablePID(Servo_t * self);
void SERVO_EnablePIV(Servo_t * self);
void SERVO_EnablePositionFilter(Servo_t * self);
void SERVO_EnableStepDirection(Servo_t * self);
bool SERVO_IsAlignmentComplete(Servo_t * self);

#endif /* APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_ */
