/*
 * user_motor_consts.h
 *
 *  Created on: May 16, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_MOTOR_CONSTS_H_
#define APPLICATION_INCLUDE_USER_MOTOR_CONSTS_H_

typedef struct {
    float PhaseResistance;
    float PhaseInductance;
} MotorConstants_t;

typedef struct {
    MotorConstants_t Consts;    
    float ResistanceSamples;
    float InductanceSamples;
    float PrevCurrent;
    float PrevVoltage;
} MotorConstantEstimator_t;

void MCONSTS_UpdateEstimates(MotorConstants_t *self) {
    float Current = (float)MC_GetPhaseCurrentAmplitudeMotor1();
    float Voltage = (float)MC_GetPhaseVoltageAmplitudeMotor1();

    // Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
    // PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]


    float Amps  = Current * 3.3f / (65536.0f * 1.0f * 40.0f);
    float Volts = Voltage * 24.0f/ (32767.0f * 1.73205f);

    self->Consts.PhaseResistance += Volts/Amps;

    self->ResistanceSamples += 1;
}

#endif /* APPLICATION_INCLUDE_USER_MOTOR_CONSTS_H_ */
