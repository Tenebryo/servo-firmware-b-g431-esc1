/*
 * user_servo_controller.c
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#include "user_servo_controller.h"

#include "mc_api.h"
#include "user_step_dir.h"

/// This initializes the servo given its current configuration data.
void SERVO_Init(Servo_t * self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t * TorqueController, FPID_Handle_t *PIDPosRegulator, FPID_Handle_t *PIDVelRegulator) {

  if (self->State == UNINIT) {

    self->Config.InputFiltKi = 2.0f * self->Config.TorqueBandwidth;
    self->Config.InputFiltKp = self->Config.TorqueBandwidth * self->Config.TorqueBandwidth;

    self->Encoder = Encoder;
    self->TorqueController = TorqueController;
    self->PIDPosRegulator = PIDPosRegulator;
    self->PIDVelRegulator = PIDVelRegulator;

    self->PIDPosRegulator = PIDPosRegulator;
    self->PIDVelRegulator = PIDVelRegulator;

    self->State = DISABLED;
  }
}

/// This is the main servo control loop. It uses the input position (from the Step/Dir interface) to command a torque.
void SERVO_ControlPosition(Servo_t * self, float DeltaTime, float PosInput) {

  float PosActual, VelActual;
  float PosDelta, VelDelta, Accel;

  switch (self->State) {
  case UNINIT:
    // command 0 torque when the controller is uninitialized (coasting)
    self->TorSetpoint = 0;
    break;
  case DISABLED:
    // command 0 torque when the controller is disabled (coasting)
    self->TorSetpoint = 0;
    break;
  case ALIGNING:
    // command a constant speed during alignment, which is set when alignment starts.
    VelActual = (float)SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);

    self->TorSetpoint = FPI_Controller(self->PIDVelRegulator, self->VelSetpoint - VelActual);

    break;
  case ENABLED:
    // get current state and inputs
    PosActual = (float)SPD_GetMecAngle(&self->Encoder->_Super) + self->EncoderOffset;
    VelActual = (float)SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);


    // filter stepped input position
    PosDelta = PosInput - self->PosSetpoint;
    VelDelta = 0.0f     - self->VelSetpoint;

    Accel = (self->Config.InputFiltKp * PosDelta) + (self->Config.InputFiltKi * VelDelta);

    // calculate the feedforward motion terms
    self->TorSetpoint = self->Config.Inertia * Accel;
    self->VelSetpoint += DeltaTime * Accel;
    self->PosSetpoint += DeltaTime * self->VelSetpoint;

    // control position with velocity, then velocity with torque, adding in the feedforward terms

    self->VelSetpoint += FPID_Controller(self->PIDPosRegulator, (self->PosSetpoint) - PosActual);

    // clamp commanded velocity to range based on config
    if (self->VelSetpoint >  self->Config.VelMaxAbs) {self->VelSetpoint =  self->Config.VelMaxAbs;}
    if (self->VelSetpoint < -self->Config.VelMaxAbs) {self->VelSetpoint = -self->Config.VelMaxAbs;}

    self->TorSetpoint += FPI_Controller(self->PIDVelRegulator, (self->VelSetpoint) - VelActual);


    break;
  }

  // clamp commanded torque to range based on config before applying it
  if (self->TorSetpoint >  self->Config.TorMaxAbs) {self->TorSetpoint =  self->Config.TorMaxAbs;}
  if (self->TorSetpoint < -self->Config.TorMaxAbs) {self->TorSetpoint = -self->Config.TorMaxAbs;}

  // actually send the desired torque to the torque controller.
  STC_SetControlMode( self->TorqueController, STC_TORQUE_MODE );
  STC_ExecRamp( self->TorqueController, (int32_t)(self->TorSetpoint), 0 );
}

/// Helper function to drive the
void SERVO_ControlPositionFromStepDir(Servo_t * self, float DeltaTime) {
  float PosInput = self->Config.StepAngle * (float)(STEPDIR_GetInputPosition() + self->StepDirOffset);
  SERVO_ControlPosition(self, DeltaTime, PosInput);
}

/// Callback that is called when the encoder index pin is triggered.
void SERVO_ResetEncoder(Servo_t * self) {

  int32_t PosActual = SPD_GetMecAngle(&self->Encoder->_Super);

  self->EncoderOffset = -PosActual;

  self->Aligned = true;
  self->State = DISABLED;
}

/// Call this function to stop servo control. The servo will coast after calling this function
void SERVO_Disable(Servo_t * self) {
  self->State = DISABLED;

  int32_t PosActual = SPD_GetMecAngle(&self->Encoder->_Super) + self->EncoderOffset;
  int32_t VelActual = SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);

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
    int32_t PosActual = SPD_GetMecAngle(&self->Encoder->_Super) + self->EncoderOffset;

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

    int32_t PosActual = SPD_GetMecAngle(&self->Encoder->_Super) + self->EncoderOffset;
    int32_t VelActual = SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);
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
