/*
 * user_gpio_interrupt_handlers.c
 * The functions that handle each interrupt are put here.
 *
 *
 *  Created on: May 14, 2021
 *      Author: Sam Blazes
 */

#include "stm32g4xx.h"
#include "mc_api.h"
#include "main.h"
#include "mc_config.h"
#include "user_servo_controller.h"

void step_pin_interrupt(void);
void button_interrupt(void);

/*
 * @brief User GPIO interrupt handler for EXTI.
 * @param uint16_t GPIO_Pin : the pin whose interrupt was triggered
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
  case USER_BUTTON_EXTI_IRQn:
    button_interrupt();
    break;
  default:
    break;
  }
}

void EXTI9_5_IRQHandler(void)
{
  SERVO_SetEncoderOffset(&ServoHandle_M1);
}

/*
 * @brief This interrupt is triggered when the step pin is activated.
 * @param None
 * @retval None
 */

/*
 * @brief This interrupt is triggered when the daughter board button is pressed
 * @param None
 * @retval None
 */
void button_interrupt(void) {
  // emergency stop using the button
  MC_StopMotor1();
}
