/*
 * user_swd_commands.h
 *
 *  Created on: Jun 29, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_SWD_COMMANDS_H_
#define APPLICATION_INCLUDE_USER_SWD_COMMANDS_H_

#define SWD_COMMAND_BUFFER_LEN (32)

#include "stdint.h"

typedef enum {
    MotorStop,
    MotorStart,
    SetPositionControl,
    SetVelocityControl,
    SetTorqueControl,
    ClearFaultState,
    PositionCommand,
    FindUpperMotionLimit,
    FindLowerMotionLimit,
} SWDCommandType_t;

typedef struct {
    SWDCommandType_t ty;
    // command specific data
    union {
        struct {
            float position;
        } position_command;
    };
} SWDCommand_t;

typedef struct {
    uint32_t front;
    uint32_t back;
    uint32_t capacity;
    SWDCommand_t *addr;
} SWDCommandBuffer_t;

extern SWDCommandBuffer_t SWDBufferInfo;
extern SWDCommand_t SWDBuffer[SWD_COMMAND_BUFFER_LEN];

uint8_t SWD_GetNextCommand(SWDCommand_t *command);

#endif /* APPLICATION_INCLUDE_USER_SWD_COMMANDS_H_ */
