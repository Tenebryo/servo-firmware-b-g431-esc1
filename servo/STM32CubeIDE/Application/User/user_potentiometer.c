/*
 * user_potentiometer.c
 *
 *  Created on: May 14, 2021
 *      Author: Sam Blazes
 */


/* Includes ------------------------------------------------------------------*/
#include "mc_api.h"
#include "regular_conversion_manager.h"
#include "parameters_conversion.h"

/* Global variables ----------------------------------------------------------*/

uint32_t POT_LastConvTick;
uint16_t POT_LastConvValue;
uint8_t POT_Handle;

/* Private macros ------------------------------------------------------------*/

/* Function prototypes -----------------------------------------------*/
bool POT_ReadValue(uint16_t *potentiometer_value);

/* Function -----------------------------------------------*/

/*
 * Initializes the potentiometer system.
 * @p regConv : the regular conversion
 */
void POT_Init(RegConv_t * regConv) {
  POT_Handle = RCM_RegisterRegConv(regConv);
  return;
}

/*
 * Sets @p potentiometer_value to the latest value read from the potentiometer.
 * @p potentiometer_value : a pointer to a uint16_t where the value that is read will be put.
 * @returns the tick during which the last value was read.
 */
uint32_t POT_ReadValue(uint16_t *potentiometer_value) {
  /* Check regular conversion state */
  if (RCM_GetUserConvState() == RCM_USERCONV_IDLE) {

    /* if Idle, then program a new conversion request */
    RCM_RequestUserConv(POT_Handle);
  } else if (RCM_GetUserConvState() == RCM_USERCONV_EOC) {
    /* if completed, then read the captured value */
    POT_LastConvValue = RCM_GetUserConv();
    POT_LastConvTick = HAL_GetTick();

  }

  *potentiometer_value = POT_LastConvValue;

  return (POT_LastConvTick);
}
