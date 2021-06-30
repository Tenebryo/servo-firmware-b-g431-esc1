/*
 * user_swd_commands.c
 *
 *  Created on: Jun 29, 2021
 *      Author: Sam Blazes
 */

#include "user_swd_commands.h"

SWDCommandBuffer_t SWDBufferInfo = {
    .capacity = SWD_COMMAND_BUFFER_LEN,
    .front = 0,
    .back = 0,
    .addr = &SWDBuffer[0],
};
SWDCommand_t SWDBuffer[SWD_COMMAND_BUFFER_LEN] = {0};

uint8_t SWD_GetNextCommand(SWDCommand_t *command) {
    if (SWDBufferInfo.front != SWDBufferInfo.back) {
        *command = SWDBuffer[SWDBufferInfo.front];
        SWDBufferInfo.front = (SWDBufferInfo.front + 1 == SWDBufferInfo.capacity) ? 0 : SWDBufferInfo.front + 1;
        return 1;
    } else {
        return 0;
    }
}