/*
 * user_servo_controller.c
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#include "user_servo_controller.h"
#include "pmsm_motor_parameters.h"
#include "mc_config.h"

#include "mc_api.h"
#include "user_step_dir.h"

#define U16MAX (65535)
// encoder position is expressed in 1/65536 of a rotation counts.
#define TURNS_PER_ENCODER_STEP (1.0f / (float)U16MAX)
#define HZ_PER_SPEED_UNIT      (1.0f / (float)SPEED_UNIT)


float clamp_f(float x, float min, float max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}


// float abs_f(float x) {
//   return (x < 0) ? (-x) : (x);
// }


/// ==================================================================================================================
/// This initializes the servo given its current configuration data.
/// ==================================================================================================================
void SERVO_Init(Servo_t * self, ENCODER_Handle_t *Encoder, SpeednTorqCtrl_Handle_t * TorqueController, FPID_Handle_t *PIDPosRegulator, FPID_Handle_t *PIVPosRegulator, FPID_Handle_t *PIVVelRegulator) {

  if (self->State == UNINIT) {

    self->AnticoggingCalibrated = false;

    self->Config.InputFiltKi = 2.0f * self->Config.TorqueBandwidth;
    self->Config.InputFiltKp = self->Config.TorqueBandwidth * self->Config.TorqueBandwidth;

    self->LastEncoderCount = TIM4->CNT;
    self->EncoderPosition = self->LastEncoderCount;

    self->Encoder = Encoder;
    self->TorqueController = TorqueController;

    self->PIDPosRegulator = PIDPosRegulator;
    self->PIVPosRegulator = PIVPosRegulator;
    self->PIVVelRegulator = PIVVelRegulator;

    // clear integrator windup
    FPID_SetIntegralTerm(self->PIDPosRegulator, 0.0);
    FPID_SetIntegralTerm(self->PIVPosRegulator, 0.0);
    FPID_SetIntegralTerm(self->PIVVelRegulator, 0.0);

    self->State = DISABLED;
  }
}

/// ==================================================================================================================
/// This is the main servo control loop. It uses the input position (from the Step/Dir interface) to command a torque.
/// ==================================================================================================================
void SERVO_ControlPosition(Servo_t * self, float DeltaTime) {

  float PosActual, VelActual;
  float PosCmd = 0.0f, VelCmd = 0.0f, TorCmd = 0.0f;
  float PosDelta, VelDelta, Accel;
  float PrevTorCmd;
  uint16_t CogPointIndex;

  TorCmd = 0.0f;

  PrevTorCmd = self->TorSetpoint;

  // some control modes only add to the torque setpoint, so we need to clear it here.
  self->TorSetpoint = 0.0f;

  self->EncoderPosition = SPD_GetMecAngle(&ENCODER_M1._Super) + self->EncoderOffset;
  PosActual = TURNS_PER_ENCODER_STEP * (float)(self->EncoderPosition);
  VelActual = HZ_PER_SPEED_UNIT      * (float)SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);

  switch (self->State) {
  case UNINIT:
    // command 0 torque when the controller is uninitialized (coasting)
    TorCmd = 0;
    break;

  case DISABLED:
    // command 0 torque when the controller is disabled (coasting)
    TorCmd = 0;
    break;

  case ALIGNING:
    // command a constant speed during alignment, which is set when alignment starts.
    // this expression does a gradual ramp up to the final speed.
    self->VelSetpoint = self->VelSetpoint * 0.9f + self->Config.IndexScanSpeed * 0.1f;

    TorCmd = FPI_Controller(self->PIVVelRegulator, self->VelSetpoint - VelActual, DeltaTime);

    break;

  case ENABLED_PID:
    self->PosSetpoint = self->PosInput;
    TorCmd = FPID_Controller(self->PIDPosRegulator, (self->PosSetpoint) - PosActual, DeltaTime);
    break;

  case ENABLED_STEP_DIRECTION:
    self->PosInput = self->Config.TurnsPerStep * (float) (STEPDIR_GetInputPosition() + self->StepDirOffset);
    // cascade to filter the step/direction inputs

  case ENABLED_POSITION_FILTER:
    // filter stepped input position
    PosDelta = self->PosInput - self->PosSetpoint;
    VelDelta = self->VelInput - self->VelSetpoint;

    Accel = (self->Config.InputFiltKp * PosDelta) + (self->Config.InputFiltKi * VelDelta);

    // calculate the feedforward motion terms
    self->TorSetpoint = self->Config.Inertia * Accel;
    self->VelSetpoint += DeltaTime * Accel;
    self->PosSetpoint += DeltaTime * self->VelSetpoint;

    self->VelSetpoint = clamp_f(self->VelSetpoint, -self->Config.VelMaxAbs, self->Config.VelMaxAbs);
    self->TorSetpoint = clamp_f(self->TorSetpoint, -self->Config.TorMaxAbs, self->Config.TorMaxAbs);

    // cascade into PIV control with the calculated feedforward setpoints

  case ANTICOGGING_CALIBRATION:
    // // todo: do this externally
    // if (self->State == ANTICOGGING_CALIBRATION) {
    //   if (abs_f(PosActual - COG_POS(self->AnticoggingIndex)) < COG_POSITION_ERR_EPS && abs_f(VelActual) < COG_VELOCITY_ERR_EPS) {
    //     // the position is within the cog range.

    //     if (self->AnticoggingSamples < COG_POSITION_SAMPLES) {
    //       // the position has been stable, so take a sample
    //       self->AnticoggingSum += PrevTorCmd;
    //     }
    //     self->AnticoggingSamples--;

    //     if (self->AnticoggingSamples == 0) {
    //       // store the average torque (average of samples from moving forward and backward)
    //       self->Config.AntcoggingTorque[self->AnticoggingIndex] += 0.5 * self->AnticoggingSum / (float)COG_POSITION_SAMPLES;

    //       // advance to the next cog sample position
    //       self->AnticoggingSum = 0.0f;
    //       self->AnticoggingSamples = COG_POSITION_SAMPLES + COG_POSITION_STABILITY_WAIT;
    //       if (self->AnticoggingReturning) {
    //         self->AnticoggingIndex--;
    //       } else {
    //         self->AnticoggingIndex++;
    //       }

    //       // check if we are done, at the turn-around point, or carrying on
    //       if (self->AnticoggingIndex == 0 && self->AnticoggingReturning) {
    //         // we are done
    //         self->AnticoggingCalibrated = true;
    //         SERVO_Disable(self);
    //       } else if (self->AnticoggingIndex == COGGING_TORQUE_POINTS) {
    //         self->AnticoggingReturning = true;
    //         self->AnticoggingIndex--;
    //       } else {
    //         // initialize the next sample point.
    //         self->PosSetpoint = COG_POS(self->AnticoggingIndex);
    //       }
    //     }
    //   }
    // }
    // // cascade into PIV control

  case ENABLED_PIV:

    if (self->State == ENABLED_PIV) {
      // in the ENABLED_PIV state, we must zero the set points otherwise they will act like integrators
      // this needs to be in an if statement so that the feedforward terms above remain.
      self->PosSetpoint = self->PosInput;
      self->VelSetpoint = self->VelInput;
      self->TorSetpoint = self->TorInput;
    }

    PosCmd = self->PosSetpoint;
    // control position with velocity, then velocity with torque, adding in the feedforward terms

    VelCmd += FPID_Controller(self->PIVPosRegulator, PosCmd - PosActual, DeltaTime);
    

  case ENABLED_VELOCITY:

    if (self->State == ENABLED_VELOCITY) {
      self->VelSetpoint = self->VelInput;
      self->TorSetpoint = self->TorInput;
    }

    VelCmd += self->VelSetpoint;

    // limit the set velocity
    VelCmd = clamp_f(VelCmd, -self->Config.VelMaxAbs, self->Config.VelMaxAbs);

    TorCmd += FPID_Controller(self->PIVVelRegulator, VelCmd - VelActual, DeltaTime);

  case ENABLED_TORQUE:

    if (self->State == ENABLED_TORQUE) {
      self->TorSetpoint = self->TorInput;
    }

    TorCmd += self->TorSetpoint;

    // only add anticogging compensation when were aren't calibrating it
    if (self->State != ANTICOGGING_CALIBRATION && self->AnticoggingCalibrated) {
      // add anticogging feedforward term. The actual torque feedforward term is interpolated between the two
      // nearest cogging torque sample points.
      CogPointIndex = COG_INDEX(PosActual);
      TorCmd += (
        (PosActual - COG_POS(CogPointIndex))     * COG_POSITION_ERR_EPS * self->Config.AntcoggingTorque[(CogPointIndex    ) % COGGING_TORQUE_POINTS] + 
        (COG_POS(CogPointIndex + 1) - PosActual) * COG_POSITION_ERR_EPS * self->Config.AntcoggingTorque[(CogPointIndex + 1) % COGGING_TORQUE_POINTS]
      );
    }


    break;
  }

  // limit the set torque
  TorCmd = clamp_f(TorCmd, -self->Config.TorMaxAbs, self->Config.TorMaxAbs);
  // this is safe because the TorSetpoint is never integrated.
  self->TorSetpoint = TorCmd;

  // actually send the desired torque to the torque controller.
  MC_ProgramTorqueRampMotor1((int32_t)(self->TorSetpoint), 0);
}

/// ==================================================================================================================
/// Helper function for various enable routines
/// ==================================================================================================================
void SERVO_Enable(Servo_t * self) {

  // we can only enable the servo if the servo is currently disabled and aligned.
    // use the current position, velocity, and input pos as the initial set points

    self->EncoderPosition = SPD_GetMecAngle(&self->Encoder->_Super) + self->EncoderOffset;

    // clear any windup in the PID integrators
    FPID_SetIntegralTerm(self->PIDPosRegulator, 0.0f);
    FPID_SetIntegralTerm(self->PIVPosRegulator, 0.0f);
    FPID_SetIntegralTerm(self->PIVVelRegulator, 0.0f);

    float PosActual = TURNS_PER_ENCODER_STEP * (float)(self->EncoderPosition);

    self->StepDirOffset = PosActual;
    self->PosSetpoint = PosActual;
    self->VelSetpoint = 0.0f;
    self->TorSetpoint = 0.0f;

    self->PosInput = self->PosSetpoint;
    self->VelInput = self->VelSetpoint;
    self->TorInput = self->TorSetpoint;
}

/// ==================================================================================================================
/// Callback that is called when the encoder index pin is triggered.
/// Since it is called every time it should stay synced.
/// ==================================================================================================================
void SERVO_ResetEncoderOffset(Servo_t * self) {

  int32_t PosActual = SPD_GetMecAngle(&self->Encoder->_Super);

  self->EncoderOffset = -PosActual;

  self->Aligned = true;
  self->State = DISABLED;
}


/// ==================================================================================================================
/// Call this function to start the servo aligning process
/// ==================================================================================================================
void SERVO_FindEncoderIndex(Servo_t * self) {

  if (self->State == DISABLED) {
    // set revoke any previous alignment
    self->State = DISABLED;
    self->Aligned = false;

    // use the current position and scan velocity as setpoints
    int32_t PosActual = SPD_GetMecAngle(&self->Encoder->_Super) + self->EncoderOffset;
    int32_t VelActual = SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);

    self->PosSetpoint = TURNS_PER_ENCODER_STEP * PosActual;
    self->VelSetpoint = HZ_PER_SPEED_UNIT      * VelActual;

    // clear integrator windup
    FPID_SetIntegralTerm(self->PIDPosRegulator, 0.0f);
    FPID_SetIntegralTerm(self->PIVPosRegulator, 0.0f);
    FPID_SetIntegralTerm(self->PIVVelRegulator, 0.0f);

    self->State = ALIGNING;
  }
}

/// ==================================================================================================================
/// Helper function to poll whether the index pulse has been found.
/// ==================================================================================================================
bool SERVO_IsAlignmentComplete(Servo_t * self) {
  return self->Aligned;
}


/// ==================================================================================================================
/// Call this function to start the anticogging proceedure
/// ==================================================================================================================
void SERVO_CalibrateAnticogging(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {

    // Clear the cogging torque buffer.
    for (int i = 0; i < COGGING_TORQUE_POINTS; i++) {
      self->Config.AntcoggingTorque[i] = 0.0;
    }

    // initialize anticogging calibration state variables
    self->AnticoggingSamples = COG_POSITION_SAMPLES + COG_POSITION_STABILITY_WAIT;
    self->AnticoggingIndex = 0;
    self->AnticoggingSum = 0.0f;
    self->PosSetpoint = 0.0f;
    self->VelSetpoint = 0.0f;
    self->TorSetpoint = 0.0f;
    self->AnticoggingReturning = false;
    SERVO_Enable(self);
    self->State = ANTICOGGING_CALIBRATION;
  }
}

/// ==================================================================================================================
/// Helper function to poll whether the anticogging calibration proceedure has finished.
/// ==================================================================================================================
bool SERVO_IsAnticoggingCalibrationComplete(Servo_t * self) {
  return self->AnticoggingCalibrated;
}

/// ==================================================================================================================
/// Enable the servo control loop in the PID control mode
/// ==================================================================================================================
void SERVO_EnablePID(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {
    SERVO_Enable(self);
    self->State = ENABLED_PID;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the PIV control mode
/// ==================================================================================================================
void SERVO_EnableTorque(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {
    SERVO_Enable(self);
    self->State = ENABLED_TORQUE;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the PIV control mode
/// ==================================================================================================================
void SERVO_EnableVelocity(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {
    SERVO_Enable(self);
    self->State = ENABLED_VELOCITY;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the PIV control mode
/// ==================================================================================================================
void SERVO_EnablePIV(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {
    SERVO_Enable(self);
    self->State = ENABLED_PIV;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the position filter control mode
/// ==================================================================================================================
void SERVO_EnablePositionFilter(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {
    SERVO_Enable(self);
    self->State = ENABLED_POSITION_FILTER;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the step direction control mode
/// ==================================================================================================================
void SERVO_EnableStepDirection(Servo_t * self) {
  if (self->Aligned && self->State == DISABLED) {
    SERVO_Enable(self);

    float PosInput = self->Config.TurnsPerStep * (float) (STEPDIR_GetInputPosition() + self->StepDirOffset);

    self->StepDirOffset -= PosInput;
    self->State = ENABLED_STEP_DIRECTION;
  }
}

/// ==================================================================================================================
/// Call this function to stop servo control. The servo will coast after calling this function
/// ==================================================================================================================
void SERVO_Disable(Servo_t * self) {
  self->State = DISABLED;
  // clear the setpoints. these should be updated when the servo is enabled again
  self->PosSetpoint = 0.0f;
  self->VelSetpoint = 0.0f;
  self->TorSetpoint = 0.0f;
}

/// ==================================================================================================================
/// Get the current position the servo thinks it is in
/// ==================================================================================================================
float SERVO_GetPosition(Servo_t *self) {
  return TURNS_PER_ENCODER_STEP * (float)(self->EncoderPosition);
}

/// ==================================================================================================================
/// Get the current position the servo thinks it is in
/// ==================================================================================================================
float SERVO_GetVelocity(Servo_t *self) {
  return HZ_PER_SPEED_UNIT      * (float)SPD_GetAvrgMecSpeedUnit(&self->Encoder->_Super);;
}

/// ==================================================================================================================
/// Get the current position the servo thinks it is in
/// ==================================================================================================================
float SERVO_GetTorque(Servo_t *self) {
  return self->TorSetpoint;
}