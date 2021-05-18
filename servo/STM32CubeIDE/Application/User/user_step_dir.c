/*
 * user_step_dir.c
 *
 *  Created on: May 15, 2021
 *      Author: Sam Blazes
 */


#include "stdbool.h"
#include "stdint.h"
#include "stm32g4xx_hal.h"

void STEPDIR_Init(void) {

}

bool STEPDIR_ReadEnable(void) {
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}


int32_t STEPDIR_GetInputPosition(void) {
  // timer2
  uint32_t pos = TIM2->CNT;

  return ((int32_t)pos);
}
