/*
 * user_config.h
 *
 *  Created on: Jun 12, 2021
 *      Author: sunbl
 */

#ifndef APPLICATION_INCLUDE_USER_CONFIG_H_
#define APPLICATION_INCLUDE_USER_CONFIG_H_

#include "stdint.h"

#include "user_servo_controller.h"
#include "user_oscilloscope.h"
#include "user_swd_commands.h"

typedef struct {
    // magic bytes to 
    uint8_t magic[7];
    uint8_t ready;
    ServoConfig_t *servo_config;
    ServoState_t *servo_state;
    Oscilloscope_t *oscilloscope;
    SamplePoint_t *oscilloscope_data;
    SWDCommandBuffer_t *swd_command_buffer;
} ConfigPointers_t;

typedef struct {
    ServoConfig_t servo_config;
    uint32_t crc;
} ConfigFlashStorage_t;

extern ConfigPointers_t ConfigPointers;
extern ConfigFlashStorage_t *ConfigFlashStorage;

void CONFIG_Init();
void CONFIG_Load();
void CONFIG_Save();

#endif /* APPLICATION_INCLUDE_USER_CONFIG_H_ */
