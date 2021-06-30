/*
 * user_config.c
 *
 *  Created on: Jun 12, 2021
 *      Author: sunbl
 */

#include "user_config.h"

ConfigPointers_t ConfigurationData;
extern ConfigPointers_t *_ptr_list_addr;

void CONFIG_Init() {
    // Magic byte sequence exists to check if we read the correct struct
    ConfigurationData.magic[0] = 0x54;
    ConfigurationData.magic[1] = 0xA4;
    ConfigurationData.magic[2] = 0x2F;
    ConfigurationData.magic[3] = 0x6F;
    ConfigurationData.magic[4] = 0x07;
    ConfigurationData.magic[5] = 0x8A;
    ConfigurationData.magic[6] = 0x48;

    ConfigurationData.servo_config = &ServoHandle_M1.Config;
    ConfigurationData.servo_state = &ServoHandle_M1.state;
    ConfigurationData.oscilloscope = &OscilloscopeHandle_M1;
    ConfigurationData.oscilloscope_data = &OscilloscopeHandle_M1.Data[0];
    ConfigurationData.swd_command_buffer = &SWDBufferInfo;

    // write the address in a location we can always find.
    _ptr_list_addr = &ConfigurationData;

    ConfigurationData.ready = 1;
}