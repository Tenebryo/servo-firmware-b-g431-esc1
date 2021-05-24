/*
 * user_oscilloscope.h
 *
 *  Created on: May 21, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_OSCILLOSCOPE_H_
#define APPLICATION_INCLUDE_USER_OSCILLOSCOPE_H_

#define OSCILLOSCOPE_SAMPLE_COUNT    (128)
#define OSCILLOSCOPE_SAMPLE_INTERVAL (2)
#define OSCILLOSCOPE_SAMPLE_SIZE     (4)

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
    uint32_t Index;
    uint32_t Interval;
    SamplePoint_t Data[OSCILLOSCOPE_SAMPLE_COUNT];
} Oscilloscope_t;

void OSC_Init(Oscilloscope_t *self);
bool OSC_CheckInterval(Oscilloscope_t *self);
void OSC_AddPoint(Oscilloscope_t *self, SamplePoint_t data);

#endif /* APPLICATION_INCLUDE_USER_OSCILLOSCOPE_H_ */
