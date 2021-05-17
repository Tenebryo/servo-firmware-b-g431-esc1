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

typedef struct {

} calibration_t;

bool CALIBRATION_WindingResistance(calibration_t *calib);
bool CALIBRATION_WindingInductance(calibration_t *calib);


#endif /* APPLICATION_INCLUDE_USER_CALIBRATION_H_ */
