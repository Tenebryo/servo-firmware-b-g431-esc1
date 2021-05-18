/*
 * user_servo_controller.c
 *
 *  Created on: May 18, 2021
 *      Author: sunbl
 */

#include "user_servo_controller.h"

#include "mc_api.h"
#include "speed_pos_fdbk.h"
#include "user_step_dir.h"

#define RADTOS16 10430.378350470452725f
#define S16TORAD (1.0f/10430.378350470452725f)

#define TO_S16(theta) (int32_t)((theta) * RADTOS16)

/// This initializes the servo given the configuration struct.
void SERVO_Init(Servo_t * self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t * TorqueController, ServoConfig_t Config) {

  self->Config = Config;

  self->State = DISABLED;

  self->PosSetpoint = 0.0f;
  self->VelSetpoint = 0.0f;
  self->TorSetpoint = 0.0f;

  int32_t EncoderOffset, StepDirOffset;

  ENCODER_Handle_t *Encoder;
  SpeednTorqCtrl_Handle_t *TorqueController;
  PID_Handle_t *PIDPosRegulator, *PIDVelRegulator;
}

/// This is the main servo control loop. It uses the input position (from the Step/Dir interface) to command a torque.
void SERVO_ControlPosition(Servo_t * self, float DeltaTime) {

  int32_t PosActual, VelActual;
  float PosInput;
  float PosDelta, VelDelta, Accel;

  switch (self->State) {
  case DISABLED:
    // command 0 torque when the controller is disabled (coasting)
    self->TorqueSetpoint = 0;
    break;
  case ALIGNING:
    // command a constant speed during alignment, which is set when alignment starts.
    VelActual = SPD_GetAvrgMecSpeedUnit(self->Encoder);

    self->TorqueSetpoint = PID_Controller(self->PIDVelRegulator, ((int32_t)self->VelSetpoint) - VelActual);

    break;
  case ENABLED:
    // get current state and inputs
    PosActual = SPD_GetMecAngle(self->Encoder) + self->EncoderOffset;
    VelActual = SPD_GetAvrgMecSpeedUnit(self->Encoder);

    PosInput = self->Config.StepAngle * (float)(STEPDIR_GetInputPosition() + self->StepDirOffset);

    // filter stepped input position
    PosDelta = PosInput - self->PosSetpoint;
    PosDelta = 0.0f     - self->VelSetpoint;

    Accel = (self->Config.InputFiltKp * PosDelta) + (self->Config.InputFiltKi * VelDelta);

    // calculate the feedforward motion terms
    self->TorSetpoint = self->Config.Inertia * Accel;
    self->VelSetpoint = DeltaTime * Accel;
    self->PosSetpoint = DeltaTime * self->VelSetpoint;

    // control position with velocity, then velocity with torque, adding in the feedforward terms

    self->VelSetpoint += (float)PID_Controller(self->PIDPosRegulator, ((int32_t)self->PosSetpoint) - PosActual);

    // clamp commanded velocity to range based on config
    if (self->VelSetpoint >  self->Config.VelMaxAbs) {self->VelSetpoint =  self->Config.VelMaxAbs;}
    if (self->VelSetpoint < -self->Config.VelMaxAbs) {self->VelSetpoint = -self->Config.VelMaxAbs;}

    self->TorSetpoint += (float)PID_Controller(self->PIDVelRegulator, ((int32_t)self->VelSetpoint) - VelActual);


    break;
  }

  // clamp commanded torque to range based on config before applying it
  if (self->TorSetpoint >  self->Config.TorMaxAbs) {self->TorSetpoint =  self->Config.TorMaxAbs;}
  if (self->TorSetpoint < -self->Config.TorMaxAbs) {self->TorSetpoint = -self->Config.TorMaxAbs;}

  // actually send the desired torque to the torque controller.
  STC_SetControlMode( self->TorqueController, STC_TORQUE_MODE );
  STC_ExecRamp( self->TorqueController, self->TorqueSetpoint, 0 );
}

/// Callback that is called when the encoder index pin is triggered.
void SERVO_ResetEncoder(Servo_t * self) {

  int32_t PosActual = SPD_GetMecAngle(self->Encoder);

  self->EncoderOffset = -PosActual;

  self->Aligned = true;
  self->State = DISABLED;
}

/// Call this function to stop servo control. The servo will coast after calling this function
void SERVO_Disable(Servo_t * self) {
  self->State = DISABLED;

  int32_t PosActual = SPD_GetMecAngle(self->Encoder) + self->EncoderOffset;
  int32_t VelActual = SPD_GetAvrgMecSpeedUnit(self->Encoder);

  self->PosSetpoint = PosActual;
  self->VelSetpoint = VelActual;
  self->TorSetpoint = 0.0f;
}

/// Call this function to start the servo aligning process
void SERVO_Align(Servo_t * self) {

  if (self->State == DISABLED) {
    // set revoke any previous alignment
    self->State = DISABLED;
    self->Aligned = false;

    // use the current position and scan velocity as setpoints
    int32_t PosActual = SPD_GetMecAngle(self->Encoder) + self->EncoderOffset;

    self->PosSetpoint = PosActual;
    self->VelSetpoint = self->Config.IndexScanSpeed;

    self->State = ALIGNING;
  }
}

/// Call this function to start the servo loop
void SERVO_Enable(Servo_t * self) {

  // we can only enable the servo if the servo is currently disabled and aligned.
  if (self->Aligned && self->State == DISABLED) {
    // use the current position, velocity, and input pos as the initial set points

    int32_t PosActual = SPD_GetMecAngle(self->Encoder) + self->EncoderOffset;
    int32_t VelActual = SPD_GetAvrgMecSpeedUnit(self->Encoder);
    int32_t InputPos = STEPDIR_GetInputPosition();

    self->PosSetpoint = PosActual;
    self->StepDirOffset = PosActual - InputPos;
    self->VelSetpoint = VelActual;

    self->State = ENABLED;
  }
}

/// Help
bool SERVO_IsAlignmentComplete(Servo_t * self) {
  return self->Aligned;
}
