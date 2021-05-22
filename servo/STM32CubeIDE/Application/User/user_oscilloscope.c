/*
 * user_oscilloscope.c
 *
 *  Created on: May 21, 2021
 *      Author: Sam Blazes
 */

#include "user_oscilloscope.h"


void OSC_Init(Oscilloscope_t *self) {
    self->Interval = 0;
    self->Index = 0;
}


bool OSC_CheckInterval(Oscilloscope_t *self) {

    if (self->Interval == 0) {
        self->Interval = OSCILLOSCOPE_SAMPLE_INTERVAL;
        return true;
    } else {
        self->Interval--;
        return false;
    }

}


void OSC_AddPoint(Oscilloscope_t *self, SamplePoint_t DataPoint) {

    self->Data[self->Index] = DataPoint;
    self->Index++;

    if (self->Index >= OSCILLOSCOPE_SAMPLE_COUNT) {
        self->Index = 0;
    }

}