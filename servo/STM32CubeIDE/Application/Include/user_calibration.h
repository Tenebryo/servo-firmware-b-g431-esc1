/*
 * user_calibration.h
 *
 *  Created on: May 16, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_CALIBRATION_H_
#define APPLICATION_INCLUDE_USER_CALIBRATION_H_

#include "mc_interface.h"

#define COGGING_TORQUE_STEPS (1)
#define MOTION_CALIBRATION_STEPS (1)

typedef struct {
  float PhaseResistance;
  float PhaseInductanceQ;
  float PhaseInductanceD;
  float AnticoggingTorque[COGGING_TORQUE_STEPS];
} MotorCalibration_t;

typedef struct {
  float LowerPositionLimit;
  float UpperPositionLimit;
  float StaticTorque[MOTION_CALIBRATION_STEPS];
  float Inertia;
  float AccelerationBandwidth;
} MotionCalibration_t;

typedef struct {

  MotorCalibration_t MotorCalib;
  MotionCalibration_t MotionCalib;
} Calibration_t;

extern Calibration_t CalibrationHandle_M1;

bool CALIB_MeasureElectricCharacteristics(float Current);

void CALIB_UpdateIqdPIGains(float CurrentBandwidth);

bool CALIB_MeasureAnticoggingTorque(float CalibrationMaxTime, float MaxPositionErr, float MaxVelocityErr, uint32_t StableSamples, uint32_t IgnoredStabilizingSamples);

void CALIB_MeasurePositionMinimum(float SearchTime, float SearchSpeed);
void CALIB_MeasurePositionMaximum(float SearchTime, float SearchSpeed);
void CALIB_MeasureInertia(float Torque, float AccelTime);

#endif /* APPLICATION_INCLUDE_USER_CALIBRATION_H_ */
