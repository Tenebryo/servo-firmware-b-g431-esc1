/*
 * user_calibration.c
 *
 *  Created on: May 16, 2021
 *      Author: Sam Blazes
 */

#include "mc_config.h"

/// given phase resistance and inductances (one each for the torque and flux current vectors), tune the
void CALIBRATION_UpdateIqdPIGains(float cc_bandwidth, float phase_r, float phase_lq, float phase_ld) {

  float pq_gain = cc_bandwidth * phase_lq;
  float pd_gain = cc_bandwidth * phase_ld;
  float idq_gain = cc_bandwidth * phase_r;

  // fixed for now, should be good enough for most values
  int16_t KpDiv = PIDIqHandle_M1.hKpDivisor;
  int16_t KiDiv = PIDIqHandle_M1.hKiDivisor;

  PIDIqHandle_M1.hKpGain = (int16_t) (pq_gain * KpDiv);
  PIDIqHandle_M1.hKiGain = (int16_t) (idq_gain * KiDiv);

  PIDIdHandle_M1.hKpGain = (int16_t) (pd_gain * KpDiv);
  PIDIdHandle_M1.hKiGain = PIDIqHandle_M1.hKiGain;
}
