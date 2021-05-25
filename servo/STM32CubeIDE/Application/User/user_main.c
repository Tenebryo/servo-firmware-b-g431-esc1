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

float Middle = 0.0;

void MAIN_Init(void) {

  HAL_Delay(500);

  // MC_ProgramSpeedRampMotor1(0, 0);
  MC_ProgramTorqueRampMotor1(0, 0);
  MC_StartMotor1();

  // HAL_Delay(10000);

  // SERVO_Align(&ServoHandle_M1);

  // while (!SERVO_IsAlignmentComplete(&ServoHandle_M1)) {}
  while (MC_GetSTMStateMotor1() != RUN) {}

  HAL_Delay(500);

  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  SERVO_ResetEncoderOffset(&ServoHandle_M1);

  CALIB_MeasurePositionMinimum(20000.0f, 1.5f);
  CALIB_MeasurePositionMaximum(20000.0f, 1.5f);

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



}

State_t state;
bool SetpointToggle = false;

void MAIN_Loop(void) {

  HAL_Delay(1);

  // uint16_t PotRawValue = 0;
  // POT_ReadValue(&PotRawValue);

  // float PotValue = ((float) PotRawValue) * (1.0f / 65536.0f);

  // ServoHandle_M1.PosInput = 
  //   (       PotValue) * CalibrationHandle_M1.MotionCalib.LowerPositionLimit + 
  //   (1.0f - PotValue) * CalibrationHandle_M1.MotionCalib.UpperPositionLimit;

  HAL_Delay(200);

  if (SetpointToggle) {
    ServoHandle_M1.PosInput = Middle + 1.0;
  } else {
    ServoHandle_M1.PosInput = Middle - 1.0;
  }
  SetpointToggle = !SetpointToggle;

  state = MC_GetSTMStateMotor1();
  if (state == FAULT_NOW || state == FAULT_OVER) {

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
