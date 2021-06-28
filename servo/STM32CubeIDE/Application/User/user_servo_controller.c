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
#define TURNS_PER_ENCODER_COUNT (1.0f / 60000.0)
#define SERVO_LOOP_DT           (0.0005f)
#define SERVO_LOOP_HZ           (2000.0f)




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

  if (self->state.State == UNINIT) {

    self->state.AnticoggingCalibrated = false;

    SERVO_UpdatePositionFilter(self);

    self->state.LastEncoderCount = TIM4->CNT;
    self->state.EncoderPosition = self->state.LastEncoderCount;

    self->Encoder = Encoder;
    self->TorqueController = TorqueController;

    self->PIDPosRegulator = PIDPosRegulator;
    self->PIVPosRegulator = PIVPosRegulator;
    self->PIVVelRegulator = PIVVelRegulator;

    self->state.Position = 0.0;
    self->state.RawPosition = 0.0;
    self->state.Velocity = 0.0;
    self->state.Accel = 0.0;

    // clear integrator windup
    FPID_SetIntegralTerm(self->PIDPosRegulator, 0.0);
    FPID_SetIntegralTerm(self->PIVPosRegulator, 0.0);
    FPID_SetIntegralTerm(self->PIVVelRegulator, 0.0);

    self->state.State = DISABLED;
  }
}

void SERVO_UpdatePositionFilter(Servo_t *self) {
    self->Config.InputFiltKi = 2.0f * self->Config.TorqueBandwidth;
    self->Config.InputFiltKp = self->Config.TorqueBandwidth * self->Config.TorqueBandwidth;
}

float abs_f(float x);

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

  PrevTorCmd = self->state.TorSetpoint;

  self->state.EncoderPosition = TIM4->CNT;

  // some control modes only add to the torque setpoint, so we need to clear it here.
  self->state.TorSetpoint = 0.0f;

  // self->state.EncoderPosition = SPD_GetMecAngle(&ENCODER_M1._Super) + self->state.EncoderOffset;
  // self->state.EncoderPosition = SPD_GetMecAngle(&ENCODER_M1._Super) + self->state.EncoderOffset;
  // VelActual = HZ_PER_SPEED_UNIT      * (float)SPD_GetAvrgMecSpeedUnit(&self->state.Encoder->_Super);
  // PosActual = TURNS_PER_ENCODER_COUNT * (float)(self->state.EncoderPosition);

  PosActual = self->state.Position;
  VelActual = self->state.Velocity;

  switch (self->state.State) {
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
    self->state.VelSetpoint = self->state.VelSetpoint * 0.9f + self->Config.IndexScanSpeed * 0.1f;

    TorCmd = FPI_Controller(self->PIVVelRegulator, self->state.VelSetpoint - VelActual, DeltaTime);

    break;

  case ENABLED_PID:
    self->state.PosSetpoint = self->state.PosInput;
    TorCmd = FPID_Controller(self->PIDPosRegulator, (self->state.PosSetpoint) - PosActual, DeltaTime);
    break;

  case ENABLED_STEP_DIRECTION:
    self->state.PosInput = self->Config.TurnsPerStep * (float) (STEPDIR_GetInputPosition() + self->state.StepDirOffset);
    // cascade to filter the step/direction inputs

  case ENABLED_POSITION_FILTER:
    // filter stepped input position
    PosDelta = self->state.PosInput - self->state.PosSetpoint;
    VelDelta = self->state.VelInput - self->state.VelSetpoint;

    if (PosDelta >  self->Config.MaxPosStep) {PosDelta =  self->Config.MaxPosStep;}
    if (PosDelta < -self->Config.MaxPosStep) {PosDelta = -self->Config.MaxPosStep;}

    Accel = (self->Config.InputFiltKp * PosDelta) + (self->Config.InputFiltKi * VelDelta);

    if (abs_f(Accel * self->Config.Inertia) > self->Config.TorMaxAbs) {
      Accel *= self->Config.TorMaxAbs / abs_f(Accel * self->Config.Inertia);
    }

    // Limit accel to prevent overspeed
    if (abs_f(DeltaTime * Accel + self->state.VelSetpoint) > self->Config.VelMaxAbs ) {
      // multiply by factor (max delta) / (proposed delta)
      Accel *= (self->Config.VelMaxAbs - abs_f(self->state.VelSetpoint)) / abs_f(DeltaTime * Accel);
    }

    // calculate the feedforward motion terms
    self->state.TorSetpoint = self->Config.Inertia * Accel; // simple filter to limit jerk
    self->state.VelSetpoint += DeltaTime * Accel;
    self->state.PosSetpoint += DeltaTime * self->state.VelSetpoint;

    self->state.VelSetpoint = clamp_f(self->state.VelSetpoint, -self->Config.VelMaxAbs, self->Config.VelMaxAbs);
    self->state.TorSetpoint = clamp_f(self->state.TorSetpoint, -self->Config.TorMaxAbs, self->Config.TorMaxAbs);

    // cascade into PIV control with the calculated feedforward setpoints

  case ANTICOGGING_CALIBRATION:
    // // todo: do this externally
    // if (self->state.State == ANTICOGGING_CALIBRATION) {
    //   if (abs_f(PosActual - COG_POS(self->state.AnticoggingIndex)) < COG_POSITION_ERR_EPS && abs_f(VelActual) < COG_VELOCITY_ERR_EPS) {
    //     // the position is within the cog range.

    //     if (self->state.AnticoggingSamples < COG_POSITION_SAMPLES) {
    //       // the position has been stable, so take a sample
    //       self->state.AnticoggingSum += PrevTorCmd;
    //     }
    //     self->state.AnticoggingSamples--;

    //     if (self->state.AnticoggingSamples == 0) {
    //       // store the average torque (average of samples from moving forward and backward)
    //       self->Config.AntcoggingTorque[self->state.AnticoggingIndex] += 0.5 * self->state.AnticoggingSum / (float)COG_POSITION_SAMPLES;

    //       // advance to the next cog sample position
    //       self->state.AnticoggingSum = 0.0f;
    //       self->state.AnticoggingSamples = COG_POSITION_SAMPLES + COG_POSITION_STABILITY_WAIT;
    //       if (self->state.AnticoggingReturning) {
    //         self->state.AnticoggingIndex--;
    //       } else {
    //         self->state.AnticoggingIndex++;
    //       }

    //       // check if we are done, at the turn-around point, or carrying on
    //       if (self->state.AnticoggingIndex == 0 && self->state.AnticoggingReturning) {
    //         // we are done
    //         self->state.AnticoggingCalibrated = true;
    //         SERVO_Disable(self);
    //       } else if (self->state.AnticoggingIndex == COGGING_TORQUE_POINTS) {
    //         self->state.AnticoggingReturning = true;
    //         self->state.AnticoggingIndex--;
    //       } else {
    //         // initialize the next sample point.
    //         self->state.PosSetpoint = COG_POS(self->state.AnticoggingIndex);
    //       }
    //     }
    //   }
    // }
    // // cascade into PIV control

  case ENABLED_PIV:

    if (self->state.State == ENABLED_PIV) {
      // in the ENABLED_PIV state, we must zero the set points otherwise they will act like integrators
      // this needs to be in an if statement so that the feedforward terms above remain.
      self->state.PosSetpoint = self->state.PosInput;
      self->state.VelSetpoint = self->state.VelInput;
      self->state.TorSetpoint = self->state.TorInput;
    }

    PosCmd = self->state.PosSetpoint;
    // control position with velocity, then velocity with torque, adding in the feedforward terms

    VelCmd += FPID_Controller(self->PIVPosRegulator, PosCmd - PosActual, DeltaTime);
    

  case ENABLED_VELOCITY:

    if (self->state.State == ENABLED_VELOCITY) {
      self->state.VelSetpoint = self->state.VelInput;
      self->state.TorSetpoint = self->state.TorInput;
    }

    VelCmd += self->state.VelSetpoint;

    // limit the set velocity
    VelCmd = clamp_f(VelCmd, -self->Config.VelMaxAbs, self->Config.VelMaxAbs);

    TorCmd += FPID_Controller(self->PIVVelRegulator, VelCmd - VelActual, DeltaTime);

  case ENABLED_TORQUE:

    if (self->state.State == ENABLED_TORQUE) {
      self->state.TorSetpoint = self->state.TorInput;
    }

    TorCmd += self->state.TorSetpoint;

    // only add anticogging compensation when were aren't calibrating it
    if (self->state.State != ANTICOGGING_CALIBRATION && self->state.AnticoggingCalibrated) {
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
  self->state.TorSetpoint = TorCmd;

  // actually send the desired torque to the torque controller.
  MC_ProgramTorqueRampMotor1((int32_t)(self->state.TorSetpoint), 0);
}

/// ==================================================================================================================
/// Helper function for various enable routines
/// ==================================================================================================================
void SERVO_Enable(Servo_t * self) {

  // we can only enable the servo if the servo is currently disabled and aligned.
  // use the current position, velocity, and input pos as the initial set points

  // self->state.EncoderPosition = SPD_GetMecAngle(&self->state.Encoder->_Super) + self->state.EncoderOffset;
  // self->state.EncoderPosition = SPD_GetMecAngle(&self->state.Encoder->_Super) + self->state.EncoderOffset;

  // clear any windup in the PID integrators
  FPID_SetIntegralTerm(self->PIDPosRegulator, 0.0f);
  FPID_SetIntegralTerm(self->PIVPosRegulator, 0.0f);
  FPID_SetIntegralTerm(self->PIVVelRegulator, 0.0f);

  // float PosActual = TURNS_PER_ENCODER_STEP * (float)(self->state.EncoderPosition);

  // self->state.StepDirOffset = PosActual;
  self->state.PosSetpoint = self->state.Position;
  self->state.VelSetpoint = 0.0f;
  self->state.TorSetpoint = 0.0f;

  self->state.PosInput = self->state.PosSetpoint;
  self->state.VelInput = self->state.VelSetpoint;
  self->state.TorInput = self->state.TorSetpoint;
}

/// ==================================================================================================================
/// Callback that is called when the encoder index pin is triggered.
/// Since it is called every time it should stay synced.
/// ==================================================================================================================
void SERVO_ResetEncoderOffset(Servo_t * self) {

  // int32_t PosActual = SPD_GetMecAngle(&self->state.Encoder->_Super);

  self->state.EncoderOffset = -(TIM4->CNT);

  self->state.Aligned = true;
  self->state.State = DISABLED;
}


/// ==================================================================================================================
/// Call this function to start the servo aligning process
/// ==================================================================================================================
void SERVO_FindEncoderIndex(Servo_t * self) {

  if (self->state.State == DISABLED) {
    // set revoke any previous alignment
    self->state.State = DISABLED;
    self->state.Aligned = false;

    // use the current position and scan velocity as setpoints
    // int32_t PosActual = SPD_GetMecAngle(&self->state.Encoder->_Super) + self->state.EncoderOffset;
    // int32_t VelActual = SPD_GetAvrgMecSpeedUnit(&self->state.Encoder->_Super);

    // self->state.PosSetpoint = TURNS_PER_ENCODER_STEP * PosActual;
    // self->state.VelSetpoint = HZ_PER_SPEED_UNIT      * VelActual;

    self->state.PosSetpoint = self->state.Position;
    self->state.VelSetpoint = self->state.Velocity;

    // clear integrator windup
    FPID_SetIntegralTerm(self->PIDPosRegulator, 0.0f);
    FPID_SetIntegralTerm(self->PIVPosRegulator, 0.0f);
    FPID_SetIntegralTerm(self->PIVVelRegulator, 0.0f);

    self->state.State = ALIGNING;
  }
}

/// ==================================================================================================================
/// Helper function to poll whether the index pulse has been found.
/// ==================================================================================================================
bool SERVO_IsAlignmentComplete(Servo_t * self) {
  return self->state.Aligned;
}


/// ==================================================================================================================
/// Call this function to start the anticogging proceedure
/// ==================================================================================================================
void SERVO_CalibrateAnticogging(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {

    // Clear the cogging torque buffer.
    for (int i = 0; i < COGGING_TORQUE_POINTS; i++) {
      self->Config.AntcoggingTorque[i] = 0.0;
    }

    // initialize anticogging calibration state variables
    self->state.AnticoggingSamples = COG_POSITION_SAMPLES + COG_POSITION_STABILITY_WAIT;
    self->state.AnticoggingIndex = 0;
    self->state.AnticoggingSum = 0.0f;
    self->state.PosSetpoint = 0.0f;
    self->state.VelSetpoint = 0.0f;
    self->state.TorSetpoint = 0.0f;
    self->state.AnticoggingReturning = false;
    SERVO_Enable(self);
    self->state.State = ANTICOGGING_CALIBRATION;
  }
}

/// ==================================================================================================================
/// Helper function to poll whether the anticogging calibration proceedure has finished.
/// ==================================================================================================================
bool SERVO_IsAnticoggingCalibrationComplete(Servo_t * self) {
  return self->state.AnticoggingCalibrated;
}

/// ==================================================================================================================
/// Enable the servo control loop in the PID control mode
/// ==================================================================================================================
void SERVO_EnablePID(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {
    SERVO_Enable(self);
    self->state.State = ENABLED_PID;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the PIV control mode
/// ==================================================================================================================
void SERVO_EnableTorque(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {
    SERVO_Enable(self);
    self->state.State = ENABLED_TORQUE;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the PIV control mode
/// ==================================================================================================================
void SERVO_EnableVelocity(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {
    SERVO_Enable(self);
    self->state.State = ENABLED_VELOCITY;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the PIV control mode
/// ==================================================================================================================
void SERVO_EnablePIV(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {
    SERVO_Enable(self);
    self->state.State = ENABLED_PIV;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the position filter control mode
/// ==================================================================================================================
void SERVO_EnablePositionFilter(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {
    SERVO_Enable(self);
    self->state.State = ENABLED_POSITION_FILTER;
  }
}

/// ==================================================================================================================
/// Enable the servo control loop in the step direction control mode
/// ==================================================================================================================
void SERVO_EnableStepDirection(Servo_t * self) {
  if (self->state.Aligned && self->state.State == DISABLED) {
    SERVO_Enable(self);

    self->state.StepDirOffset = -STEPDIR_GetInputPosition();
    self->state.State = ENABLED_STEP_DIRECTION;
  }
}

/// ==================================================================================================================
/// Call this function to stop servo control. The servo will coast after calling this function
/// ==================================================================================================================
void SERVO_Disable(Servo_t * self) {
  self->state.State = DISABLED;
  // clear the setpoints. these should be updated when the servo is enabled again
  self->state.PosSetpoint = 0.0f;
  self->state.VelSetpoint = 0.0f;
  self->state.TorSetpoint = 0.0f;
}

/// ==================================================================================================================
/// Get the current position the servo thinks it is in
/// ==================================================================================================================
float SERVO_GetPosition(Servo_t *self) {
  // return TURNS_PER_ENCODER_STEP * (float)(self->state.EncoderPosition);
  return self->state.Position;
}

/// ==================================================================================================================
/// Get the current position the servo thinks it is in
/// ==================================================================================================================
float SERVO_GetVelocity(Servo_t *self) {
  // return HZ_PER_SPEED_UNIT      * (float)SPD_GetAvrgMecSpeedUnit(&self->state.Encoder->_Super);;
  return self->state.Velocity;
}

/// ==================================================================================================================
/// Get the current position the servo thinks it is in
/// ==================================================================================================================
float SERVO_GetTorque(Servo_t *self) {
  return self->state.TorSetpoint;
}

/// ==================================================================================================================
/// Calculate the current servo position and velocity
/// ==================================================================================================================
void SERVO_UpdateDynamicState(Servo_t *self) {
  float PrevPosition = self->state.RawPosition;
  self->state.RawPosition = TURNS_PER_ENCODER_COUNT * TIM4->CNT;

  float InstVelocity = (self->state.RawPosition - PrevPosition);

  if (InstVelocity > 0.5f) {
    InstVelocity -= 1.0f;
  }
  
  if (InstVelocity < -0.5f) {
    InstVelocity += 1.0f;
  }

  self->state.Position += InstVelocity;

  InstVelocity *= SERVO_LOOP_HZ;

  float PrevVelocity = self->state.Velocity;

  // TODO: configurable filter parameters
  self->state.Velocity = 0.9 * self->state.Velocity + 0.1 * InstVelocity;

  float InstAccel = (self->state.Velocity - PrevVelocity) * SERVO_LOOP_HZ;

  self->state.Accel = 0.9 * self->state.Accel + 0.1 * InstAccel;

  if (self->state.Velocity >  self->state.MaxVelAbsObs) {self->state.MaxVelAbsObs =  self->state.Velocity;}
  if (self->state.Velocity < -self->state.MaxVelAbsObs) {self->state.MaxVelAbsObs = -self->state.Velocity;}
  
  if (self->state.Accel >  self->state.MaxAccAbsObs) {self->state.MaxAccAbsObs =  self->state.Accel;}
  if (self->state.Accel < -self->state.MaxAccAbsObs) {self->state.MaxAccAbsObs = -self->state.Accel;}
}


FPID_Handle_t PIDPosHandle_M1 = {
  
  .hDefKpGain          =  4000.0,
  .hDefKiGain          =     0.0,
  .hDefKdGain          = 16000.0,
  .wUpperIntegralLimit =  2000.0,
  .wLowerIntegralLimit = -2000.0,
  .hUpperOutputLimit   =  8000.0,
  .hLowerOutputLimit   = -8000.0,
};

FPID_Handle_t PIVPosHandle_M1 = {
  
  .hDefKpGain          =   200.0,
  .hDefKiGain          =     0.0,
  .hDefKdGain          =     0.0,
  .wUpperIntegralLimit =   100.0,
  .wLowerIntegralLimit =  -100.0,
  .hUpperOutputLimit   =   100.0,
  .hLowerOutputLimit   =  -100.0,
};

FPID_Handle_t PIVVelHandle_M1 = {
  
  .hDefKpGain          =     2000.0,
  .hDefKiGain          =        1.0,
  .hDefKdGain          =        0.0,
  .wUpperIntegralLimit =      500.0,
  .wLowerIntegralLimit =     -500.0,
  .hUpperOutputLimit   =     6000.0,
  .hLowerOutputLimit   =    -6000.0,
};

Servo_t ServoHandle_M1 =
{
    .Config = {
      .IndexScanSpeed  =     3.0,
      .TurnsPerStep    =     1.0 / 60000.0,
      .Inertia         =     1.0,
      .TorqueBandwidth =   200.0,
      .VelMaxAbs       =   100.0,
      .TorMaxAbs       =  6000.0,
      .MaxPosStep      =     0.5, //bad news if we see too big of a step
    },
    .state = {
      .State = UNINIT,
      .PosSetpoint = 0.0f,
      .VelSetpoint = 0.0f,
      .TorSetpoint = 0.0f,
      .Aligned = false,
      .EncoderOffset = 0,
      .StepDirOffset = 0,
    }
};