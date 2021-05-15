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
#include "regular_conversion_manager.h"


#ifdef __cplusplus
 extern "C" {
#endif


 void POT_ReadValue(uint16_t *potentiometer_value);

 void POT_Init(RegConv_t * regConv);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_USER_POTENTIOMETER_H_ */
