/*
 * user_gpio_interrupt_handlers.c
 * The functions that handle each interrupt are put here.
 *
 *
 *  Created on: May 14, 2021
 *      Author: Sam Blazes
 */

#include "stm32g4xx_hal_gpio.h"

/*
 * @brief User GPIO interrupt handler for EXTI.
 * @param uint16_t GPIO_Pin : the pin whose interrupt was triggered
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
  case GPIO_PIN_10:
    button_interrupt();
  case GPIO_PIN_15:
    step_pin_interrupt();
    break;
  default:
    break;
  }
}


typedef struct {

} step_dir_filter;


/*
 * @brief This interrupt is triggered when the step pin is activated.
 * @param None
 * @retval None
 */
void step_pin_interrupt() {
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) {

  } else {

  }
}

/*
 * @brief This interrupt is triggered when the daughter board button is pressed
 * @param None
 * @retval None
 */
void button_interrupt() {

}
