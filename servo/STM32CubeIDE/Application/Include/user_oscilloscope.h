/*
 * user_oscilloscope.h
 *
 *  Created on: May 21, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_OSCILLOSCOPE_H_
#define APPLICATION_INCLUDE_USER_OSCILLOSCOPE_H_

#define OSCILLOSCOPE_SAMPLE_COUNT    (256)
#define OSCILLOSCOPE_SAMPLE_INTERVAL (0)
#define OSCILLOSCOPE_SAMPLE_SIZE     (8)

#include "stdint.h"
#include "stdbool.h"

typedef enum {
    TICK,
    POS_MEASURED,
    VEL_MEASURED,
    POS_SETPOINT,
    VEL_SETPOINT,
    TOR_SETPOINT,
    POS_INPUT,
    VEL_INPUT,
    TOR_INPUT,
    CURRENT_IQ,
    CURRENT_ID,
    VOLTAGE_VQ,
    VOLTAGE_VD,
} DataSource_t;

typedef struct {
    float Sample[OSCILLOSCOPE_SAMPLE_SIZE];
} SamplePoint_t;

typedef struct {
    bool Recording;
    uint32_t Index;
    uint32_t Interval;
    uint32_t Length;
    SamplePoint_t Data[OSCILLOSCOPE_SAMPLE_COUNT];
} Oscilloscope_t;

extern Oscilloscope_t OscilloscopeHandle_M1;

void OSC_Init(Oscilloscope_t *self);
bool OSC_CheckInterval(Oscilloscope_t *self);
void OSC_AddPoint(Oscilloscope_t *self, SamplePoint_t *data);
void OSC_StartRecording(Oscilloscope_t *self);
void OSC_StopRecording(Oscilloscope_t *self);

#endif /* APPLICATION_INCLUDE_USER_OSCILLOSCOPE_H_ */
