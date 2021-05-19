/*
 * user_main.c
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#include "main.h"
#include "mc_api.h"

void MAIN_Init(void) {

  HAL_Delay(500);

  MC_ProgramTorqueRampMotor1(0, 0);
  MC_StartMotor1();

  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
}

void MAIN_Loop(void) {
}
