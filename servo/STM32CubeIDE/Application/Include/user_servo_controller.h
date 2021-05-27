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

#define COGGING_TORQUE_POINTS (512)
#define COG_INDEX(step) ((uint16_t) ((step) * COGGING_TORQUE_POINTS))
#define COG_POS(step) (((float)step) / (float)COGGING_TORQUE_POINTS)

#define COG_POSITION_ERR_EPS (0.25 / COGGING_TORQUE_POINTS)
#define COG_VELOCITY_ERR_EPS (1.0 / SPEED_UNIT)
#define COG_POSITION_SAMPLES (128)
#define COG_POSITION_STABILITY_WAIT (128)

typedef struct {
  float IndexScanSpeed;
  float TurnsPerStep;
  float VelMaxAbs;
  float TorMaxAbs;
  float InputFiltKp;
  float InputFiltKi;
  float Inertia;
  float TorqueBandwidth;
  float VelPLLKi;
  float AntcoggingTorque[COGGING_TORQUE_POINTS];
} ServoConfig_t;

typedef enum {
  UNINIT,
  DISABLED,
  ALIGNING,
  ANTICOGGING_CALIBRATION,
  ENABLED_STEP_DIRECTION,
  ENABLED_POSITION_FILTER,
  ENABLED_PID,
  ENABLED_PIV,
  ENABLED_VELOCITY,
  ENABLED_TORQUE,
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

  bool AnticoggingCalibrated;
  bool AnticoggingReturning;
  uint16_t AnticoggingSamples;
  uint16_t AnticoggingIndex;
  float AnticoggingSum;

  ENCODER_Handle_t *Encoder;
  SpeednTorqCtrl_Handle_t *TorqueController;
  FPID_Handle_t *PIDPosRegulator, *PIVPosRegulator, *PIVVelRegulator;
} Servo_t;


extern FPID_Handle_t PIDPosHandle_M1;
extern FPID_Handle_t PIVPosHandle_M1;
extern FPID_Handle_t PIVVelHandle_M1;
extern Servo_t ServoHandle_M1;

void SERVO_Init(Servo_t *self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t *TorqueController, FPID_Handle_t *PIDPosRegulator, FPID_Handle_t *PIVPosRegulator, FPID_Handle_t *PIVVelRegulator);
void SERVO_ControlPosition(Servo_t *self, float DeltaTime);
void SERVO_ResetEncoderOffset(Servo_t *self);
void SERVO_Disable(Servo_t *self);
void SERVO_FindEncoderIndex(Servo_t *self);
bool SERVO_IsAlignmentComplete(Servo_t *self);
void SERVO_CalibrateAnticogging(Servo_t *self);
bool SERVO_IsAnticoggingCalibrationComplete(Servo_t *self);
void SERVO_EnablePID(Servo_t *self);
void SERVO_EnablePIV(Servo_t *self);
void SERVO_EnableVelocity(Servo_t *self);
void SERVO_EnableTorque(Servo_t *self);
void SERVO_EnablePositionFilter(Servo_t *self);
void SERVO_EnableStepDirection(Servo_t *self);
void SERVO_UpdatePositionFilter(Servo_t *self);
float SERVO_GetPosition(Servo_t *self);
float SERVO_GetVelocity(Servo_t *self);
float SERVO_GetTorque(Servo_t *self);

#endif /* APPLICATION_INCLUDE_USER_SERVO_CONTROLLER_H_ */
