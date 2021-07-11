/*
 * user_main.c
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#include "main.h"
#include "mc_api.h"
#include "mc_config.h"
#include "user_servo_controller.h"
#include "trajectory_ctrl.h"
#include "user_calibration.h"
#include "user_potentiometer.h"
#include "user_config.h"
#include "user_swd_commands.h"

float Middle = 0.0;

typedef enum {
  DEBUG_START,
  DEBUG_ALIGNING,
  DEBUG_CALIBRATING,
  DEBUG_RUNNING,
} DebugState_t;

typedef enum {
  DEBUG_NOFAULT      = 0,
  DEBUG_FOC_DURATION = 1,
  DEBUG_OVER_VOLT    = 2,
  DEBUG_UNDER_VOLT   = 3,
  DEBUG_OVER_TEMP    = 4,
  DEBUG_START_UP     = 5,
  DEBUG_SPEED_FDBK   = 6,
  DEBUG_BREAK_IN     = 7,
  DEBUG_SW_ERROR     = 8,
} FaultType_t;

DebugState_t DebugState = DEBUG_START;
FaultType_t DebugFaultState = DEBUG_NOFAULT;
State_t state;
float PotentiometerOffset = 0.0;

void DebugHandleFault();

void MAIN_Init(void) {

  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
  CONFIG_Init();

  HAL_Delay(500);

  OSC_Init(&OscilloscopeHandle_M1);

  // MC_ProgramSpeedRampMotor1(0, 0);
  MC_ProgramTorqueRampMotor1(0, 0);
  MC_StartMotor1();

  // HAL_Delay(10000);
  DebugState = DEBUG_ALIGNING;
  // SERVO_Align(&ServoHandle_M1);

  // while (!SERVO_IsAlignmentComplete(&ServoHandle_M1)) {}
  while (MC_GetSTMStateMotor1() != RUN) { DebugHandleFault(); }

  HAL_Delay(500);

  DebugState = DEBUG_CALIBRATING;

  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  SERVO_ResetEncoderOffset(&ServoHandle_M1);

  CALIB_MeasurePositionMinimum(20000.0f, 4.0f);

  HAL_Delay(100);

  CALIB_MeasurePositionMaximum(20000.0f, 4.0f);

  // HAL_Delay(2000);

  // CALIB_MeasureInertia(2000.0f, 50.0f);

  // SERVO_FindEncoderIndex(&ServoHandle_M1);

  // SERVO_CalibrateAnticogging(&ServoHandle_M1);

  // while (!SERVO_IsAnticoggingCalibrationComplete(&ServoHandle_M1)) {}

  // ServoHandle_M1.Config.Inertia = 1.25;
  // ServoHandle_M1.Config.TorqueBandwidth = 0.5;
  // ServoHandle_M1.Config.Inertia = 1.0f * CalibrationHandle_M1.MotionCalib.Inertia;
  // ServoHandle_M1.Config.TorqueBandwidth = CalibrationHandle_M1.MotionCalib.AccelerationBandwidth;


  HAL_Delay(2000);

  // SERVO_EnablePID(&ServoHandle_M1);
  // SERVO_EnablePIV(&ServoHandle_M1);
  SERVO_EnablePositionFilter(&ServoHandle_M1);


  Middle = 0.5 * (
    CalibrationHandle_M1.MotionCalib.LowerPositionLimit +
    CalibrationHandle_M1.MotionCalib.UpperPositionLimit
  );


  ServoHandle_M1.state.MaxVelAbsObs = 0.0;
  ServoHandle_M1.state.MaxAccAbsObs = 0.0;

  float start = SERVO_GetPosition(&ServoHandle_M1);

  float end = Middle + 0.5;

  for (int i = 0; i < 1000; i++) {
    ServoHandle_M1.state.PosInput = (end * i + start * (1000 - i)) * 0.001;
    HAL_Delay(1);
  }
  HAL_Delay(1000);


  uint16_t PotRawValue = 0;
  POT_ReadValue(&PotRawValue);

  PotentiometerOffset = ((float) PotRawValue) * (1.0f / 65536.0f);

  uint32_t WaveformIndex = 32;
  const float samples[128] = {0.,0.0490677,0.0980171,0.14673,0.19509,0.24298,0.290285,0.33689,0.382683,0.427555,0.471397,0.514103,0.55557,0.595699,0.634393,0.671559,0.707107,0.740951,0.77301,0.803208,0.83147,0.857729,0.881921,0.903989,0.92388,0.941544,0.95694,0.970031,0.980785,0.989177,0.995185,0.998795,1.,0.998795,0.995185,0.989177,0.980785,0.970031,0.95694,0.941544,0.92388,0.903989,0.881921,0.857729,0.83147,0.803208,0.77301,0.740951,0.707107,0.671559,0.634393,0.595699,0.55557,0.514103,0.471397,0.427555,0.382683,0.33689,0.290285,0.24298,0.19509,0.14673,0.0980171,0.0490677,1.22465e-16,-0.0490677,-0.0980171,-0.14673,-0.19509,-0.24298,-0.290285,-0.33689,-0.382683,-0.427555,-0.471397,-0.514103,-0.55557,-0.595699,-0.634393,-0.671559,-0.707107,-0.740951,-0.77301,-0.803208,-0.83147,-0.857729,-0.881921,-0.903989,-0.92388,-0.941544,-0.95694,-0.970031,-0.980785,-0.989177,-0.995185,-0.998795,-1.,-0.998795,-0.995185,-0.989177,-0.980785,-0.970031,-0.95694,-0.941544,-0.92388,-0.903989,-0.881921,-0.857729,-0.83147,-0.803208,-0.77301,-0.740951,-0.707107,-0.671559,-0.634393,-0.595699,-0.55557,-0.514103,-0.471397,-0.427555,-0.382683,-0.33689,-0.290285,-0.24298,-0.19509,-0.14673,-0.0980171,-0.0490677,-2.44929e-16, 0.0};
  // const float samples[256] = {0.,0.015625,0.03125,0.046875,0.0625,0.078125,0.09375,0.109375,0.125,0.140625,0.15625,0.171875,0.1875,0.203125,0.21875,0.234375,0.25,0.265625,0.28125,0.296875,0.3125,0.328125,0.34375,0.359375,0.375,0.390625,0.40625,0.421875,0.4375,0.453125,0.46875,0.484375,0.5,0.515625,0.53125,0.546875,0.5625,0.578125,0.59375,0.609375,0.625,0.640625,0.65625,0.671875,0.6875,0.703125,0.71875,0.734375,0.75,0.765625,0.78125,0.796875,0.8125,0.828125,0.84375,0.859375,0.875,0.890625,0.90625,0.921875,0.9375,0.953125,0.96875,0.984375,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,0.984375,0.96875,0.953125,0.9375,0.921875,0.90625,0.890625,0.875,0.859375,0.84375,0.828125,0.8125,0.796875,0.78125,0.765625,0.75,0.734375,0.71875,0.703125,0.6875,0.671875,0.65625,0.640625,0.625,0.609375,0.59375,0.578125,0.5625,0.546875,0.53125,0.515625,0.5,0.484375,0.46875,0.453125,0.4375,0.421875,0.40625,0.390625,0.375,0.359375,0.34375,0.328125,0.3125,0.296875,0.28125,0.265625,0.25,0.234375,0.21875,0.203125,0.1875,0.171875,0.15625,0.140625,0.125,0.109375,0.09375,0.078125,0.0625,0.046875,0.03125,0.015625,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0};


  uint32_t MotionProfile = 0;

  while (1) {
    SWDCommand_t command;
    while (SWD_GetNextCommand(&command)) {
      switch (command.ty) {
      case MotorStop:
        MC_StopMotor1();
        break;
      case MotorStart:
        MC_StartMotor1();
        break;
      case SetPositionControl:
        SERVO_EnablePositionFilter(&ServoHandle_M1);
        break;
      case SetVelocityControl:
        SERVO_EnableVelocity(&ServoHandle_M1);
        break;
      case SetTorqueControl:
        SERVO_EnableTorque(&ServoHandle_M1);
        break;
      case ClearFaultState:
        MC_AcknowledgeFaultMotor1();
        MC_StopMotor1();
        break;
      case PositionCommand:
        ServoHandle_M1.state.PosInput = command.position_command.position;
        break;
      case FindUpperMotionLimit:
        ServoHandle_M1.state.VelInput = command.velocity_command.velocity;
        break;
      case FindLowerMotionLimit:
        ServoHandle_M1.state.TorInput = command.torque_command.torque;
        break;
      case LoadServoConfig:
        CONFIG_Load();
        break;
      case SaveServoConfig:
        CONFIG_Save();
        break;
      case SetMotionProfile:
        WaveformIndex = 0;
        MotionProfile = command.motion_profile_command.profile;
        break;
      default:
        break;
      }
    }

    switch (MotionProfile) {
    case 1:
      WaveformIndex += 1;
      if (WaveformIndex > 127) {
        WaveformIndex = 0;
      }

      ServoHandle_M1.state.PosInput = Middle + 0.5 * (samples[WaveformIndex]);
      HAL_Delay(1);
      break;
    default:
      break;
    }
  }
}


void MAIN_Loop(void) {

  DebugHandleFault();

}

void DebugHandleFault() {
  state = MC_GetSTMStateMotor1();
  if (state == FAULT_NOW || state == FAULT_OVER) {
    OSC_StopRecording(&OscilloscopeHandle_M1);
    uint16_t fault = MC_GetOccurredFaultsMotor1();
    MC_AcknowledgeFaultMotor1();
    MC_StopMotor1();

    while(1) {
      

      uint16_t faults[] = {
        0x0001u, // MC_FOC_DURATION 1
        0x0002u, // MC_OVER_VOLT    2
        0x0004u, // MC_UNDER_VOLT   3
        0x0008u, // MC_OVER_TEMP    4
        0x0010u, // MC_START_UP     5
        0x0020u, // MC_SPEED_FDBK   6
        0x0040u, // MC_BREAK_IN     7
        0x0080u, // MC_SW_ERROR     8
      };

      for (uint16_t i = 0; i < 8; i++) {
        if (faults[i] & fault) {
          DebugFaultState = i+1;
          for (uint16_t j = 0; j < 2*(i+1); j++) {
            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
            HAL_Delay(400);
          }
          HAL_Delay(2000);
        }
      }

    }
  }
}
