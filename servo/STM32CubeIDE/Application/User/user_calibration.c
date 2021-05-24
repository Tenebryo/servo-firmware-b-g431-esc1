/*
 * user_calibration.c
 *
 *  Created on: May 16, 2021
 *      Author: Sam Blazes
 */

#include "mc_config.h"
#include "user_calibration.h"
#include "user_servo_controller.h"

Calibration_t CalibrationHandle_M1;


float abs_f(float x) {
  return (x < 0) ? (-x) : (x);
}


bool CALIB_MeasureElectricCharacteristics(float Current) {

  return false;
}

/// given phase resistance and inductances (one each for the torque and flux current vectors), tune the
void CALIBRATION_UpdateIqdPIGains(float CurrentBandwidth) {

  float pq_gain = CurrentBandwidth * CalibrationHandle_M1.MotorCalib.PhaseInductanceQ;
  float pd_gain = CurrentBandwidth * CalibrationHandle_M1.MotorCalib.PhaseInductanceD;
  float idq_gain = CurrentBandwidth * CalibrationHandle_M1.MotorCalib.PhaseResistance;

  // fixed for now, should be good enough for most motors
  int16_t KpDiv = PIDIqHandle_M1.hKpDivisor;
  int16_t KiDiv = PIDIqHandle_M1.hKiDivisor;

  PIDIqHandle_M1.hKpGain = (int16_t) (pq_gain * KpDiv);
  PIDIqHandle_M1.hKiGain = (int16_t) (idq_gain * KiDiv);

  PIDIdHandle_M1.hKpGain = (int16_t) (pd_gain * KpDiv);
  PIDIdHandle_M1.hKiGain = PIDIqHandle_M1.hKiGain;
}


bool CALIB_MeasureAnticoggingTorque(float CalibrationMaxTime, float MaxPositionErr, float MaxVelocityErr, uint32_t StableSamples, uint32_t IgnoredStabilizingSamples) {

  SERVO_EnablePIV(&ServoHandle_M1);

  uint32_t AnticoggingIndex = 0;
  uint32_t AnticoggingSamples = 0;

  float TorqueAvg = 0.0f;

  bool CompletedCalibration = false;
  bool AnticoggingReturning = false;

  while (1) {


    float PosDiff = SERVO_GetPosition(&ServoHandle_M1) - COG_POS(AnticoggingIndex);
    float VelDiff = SERVO_GetVelocity(&ServoHandle_M1) - 0.0f;

    if (abs_f(PosDiff) < MaxPositionErr && abs_f(VelDiff) < MaxVelocityErr) {
      // the position is within the cog range.

      if (AnticoggingSamples < StableSamples) {
        // the position has been stable, so take a sample
        TorqueAvg += ServoHandle_M1.TorSetpoint;
      }
      AnticoggingSamples--;

      if (AnticoggingSamples == 0) {
        // store the average torque (average of samples from moving forward and backward)
        CalibrationHandle_M1.MotorCalib.AnticoggingTorque[AnticoggingIndex] += 0.5f * TorqueAvg / (float)StableSamples;

        // advance to the next cog sample position
        TorqueAvg = 0.0f;
        AnticoggingSamples = StableSamples + IgnoredStabilizingSamples;
        if (AnticoggingReturning) {
          AnticoggingIndex--;
        } else {
          AnticoggingIndex++;
        }

        // check if we are done, at the turn-around point, or carrying on
        if (AnticoggingIndex == 0 && AnticoggingReturning) {
          // we are done
          CompletedCalibration = true;
          break;
        } else if (AnticoggingIndex == COGGING_TORQUE_STEPS) {
          AnticoggingReturning = true;
          AnticoggingIndex--;
        } else {
          // initialize the next sample point.
          ServoHandle_M1.PosInput = COG_POS(AnticoggingIndex);
        }
      }
    }

    HAL_Delay(1);
  }

  SERVO_Disable(&ServoHandle_M1);

  return CompletedCalibration;
}


void CALIB_MeasurePositionExtremum(float SearchTime, float SearchSpeed, float *Limit) {

  uint32_t StartTick = HAL_GetTick();

  SERVO_EnableVelocity(&ServoHandle_M1);

  ServoHandle_M1.VelInput = SearchSpeed;

  // wait for a bit for the initial acceleration
  HAL_Delay(500);


  float VelocityFiltered = SearchSpeed;
  while (HAL_GetTick() < StartTick + SearchTime) {

    VelocityFiltered = (0.99 * VelocityFiltered) + (0.01 * SERVO_GetVelocity(&ServoHandle_M1));

    if (abs_f(VelocityFiltered) < abs_f(SearchSpeed * 0.1)) {
      *Limit = SERVO_GetPosition(&ServoHandle_M1);
      break;
    }

    HAL_Delay(1);
  }

  SERVO_Disable(&ServoHandle_M1);
}

void CALIB_MeasurePositionMinimum(float SearchTime, float SearchSpeed) {
  CALIB_MeasurePositionExtremum(
    SearchTime, -SearchSpeed, &CalibrationHandle_M1.MotionCalib.LowerPositionLimit
  );
}

void CALIB_MeasurePositionMaximum(float SearchTime, float SearchSpeed) {
  CALIB_MeasurePositionExtremum(
    SearchTime, SearchSpeed, &CalibrationHandle_M1.MotionCalib.UpperPositionLimit
  );
}

void CALIB_MeasureInertia(float Torque, float AccelTime) {

  SERVO_EnableTorque(&ServoHandle_M1);

  ServoHandle_M1.TorInput = Torque;

  HAL_Delay(25);

  // Measure change in velocity and position over a period of constant acceleration
  float StartPosition = SERVO_GetPosition(&ServoHandle_M1);
  float StartVelocity = SERVO_GetVelocity(&ServoHandle_M1);

  HAL_Delay(AccelTime);

  float EndPosition = SERVO_GetPosition(&ServoHandle_M1);
  float EndVelocity = SERVO_GetVelocity(&ServoHandle_M1);


  ServoHandle_M1.TorInput = 0;

  HAL_Delay(100);

  float Acceleration = 0.5 * (EndVelocity * EndVelocity - StartVelocity * StartVelocity) / (EndPosition - StartPosition);

  ServoHandle_M1.TorInput = -Torque;
  HAL_Delay(AccelTime / 2);

  SERVO_Disable(&ServoHandle_M1);


  CalibrationHandle_M1.MotionCalib.Inertia = Torque / Acceleration;
  CalibrationHandle_M1.MotionCalib.AccelerationBandwidth = Acceleration * (ServoHandle_M1.Config.TorMaxAbs / Torque);
}



