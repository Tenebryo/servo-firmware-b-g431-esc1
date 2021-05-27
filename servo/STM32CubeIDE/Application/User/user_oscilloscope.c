/*
 * user_oscilloscope.c
 *
 *  Created on: May 21, 2021
 *      Author: Sam Blazes
 */

#include "user_oscilloscope.h"


Oscilloscope_t OscilloscopeHandle_M1 = {
  .Index = 0,
  .Interval = 0,
};

void OSC_Init(Oscilloscope_t *self) {
    self->Interval = 0;
    self->Index = 0;
    self->Recording = true;
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


void OSC_AddPoint(Oscilloscope_t *self, SamplePoint_t *DataPoint) {

    if (self->Recording) {
        self->Data[self->Index].Sample[0] = DataPoint->Sample[0];
        self->Data[self->Index].Sample[1] = DataPoint->Sample[1];
        self->Data[self->Index].Sample[2] = DataPoint->Sample[2];
        self->Data[self->Index].Sample[3] = DataPoint->Sample[3];
        self->Index++;

        if (self->Index >= OSCILLOSCOPE_SAMPLE_COUNT) {
            self->Index = 0;
        }
    }

}


void OSC_StartRecording(Oscilloscope_t *self) {
    self->Recording = true;
}

void OSC_StopRecording(Oscilloscope_t *self) {
    self->Recording = false;
}