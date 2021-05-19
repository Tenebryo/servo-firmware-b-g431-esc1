/*
 * user_calibration.h
 *
 *  Created on: May 16, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_CALIBRATION_H_
#define APPLICATION_INCLUDE_USER_CALIBRATION_H_

#include "mc_interface.h"

typedef struct {
    uint32_t rs;
    uint32_t ls;
} motorCharacteristics_t;

#define COGGING_TORQUE_STEPS

typedef struct {
  uint16_t PhaseResistance;
  uint16_t PhaseInductanceQ;
  uint16_t PhaseInductanceD;
  int16_t CoggingTorque[COGGING_TORQUE_STEPS];
} calibration_t;


bool CALIBRATION_MeasureWindingResistance(calibration_t *calib);
bool CALIBRATION_MeasureWindingInductance(calibration_t *calib);
bool CALIBRATION_MeasureCoggingTorque(calibration_t *calib);

void CALIBRATION_UpdateIqdPIGains(float cc_bandwidth, float phase_r, float phase_lq, float phase_ld);

#endif /* APPLICATION_INCLUDE_USER_CALIBRATION_H_ */
