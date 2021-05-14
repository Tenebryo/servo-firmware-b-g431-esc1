/*
 * user_potentiometer.h
 *
 *  Created on: May 14, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_USER_USER_POTENTIOMETER_H_
#define APPLICATION_USER_USER_POTENTIOMETER_H_


#include "mc_api.h"
#include "parameters_conversion.h"


#ifdef __cplusplus
 extern "C" {
#endif


 void POT_ReadValue(uint16_t *potentiometer_value);

 void POT_Init(uint8_t handle);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_USER_POTENTIOMETER_H_ */
