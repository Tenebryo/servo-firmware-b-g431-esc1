/*
 * user_state_machine.c
 *
 *  Created on: May 15, 2021
 *      Author: Sam Blazes
 */


#include "user_step_dir.h"

uint16_t STATEMACHINE_State = 0;
bool STATEMACHINE_IsCalibrated = false;
uint32_t STATEMACHINE_StateFrame = 0;

#define STATE_IDLE              (0)
#define STATE_POSITION_CONTROL  (1)
#define STATE_CALIBRATION_START (2)

void SetState(uint16_t new_state) {
  STATEMACHINE_State = new_state;
  STATEMACHINE_StateFrame = -1;
}

void STATEMACHINE_Update() {

  switch (STATEMACHINE_State) {
  case STATE_IDLE:
    if (STEPDIR_ReadEnable()) {
     // motor was just enabled, check calibration and either calibrate or servo
      if (STATEMACHINE_IsCalibrated) {
        SetState(STATE_POSITION_CONTROL);
      } else {
        SetState(STATE_CALIBRATION_START);
      }
    }
    break;
  case STATE_CALIBRATION_START:

    break;
  case STATE_POSITION_CONTROL:
    break;
  }

  STATEMACHINE_StateFrame += 1;

}
