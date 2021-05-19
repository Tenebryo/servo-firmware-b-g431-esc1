/*
 * user_fpid_regulator.c
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#include "user_fpid_regulator.h"

#include "mc_type.h"

#define FLOAT_MAX (1e10)
#define EPS (0.00001)
#define FEQ(a,b) (-EPS < ((a) - (b)) && ((a) - (b)) < EPS)

void FPID_HandleInit( FPID_Handle_t * pHandle )
{
  pHandle->hKpGain =  pHandle->hDefKpGain;
  pHandle->hKiGain =  pHandle->hDefKiGain;
  pHandle->hKdGain =  pHandle->hDefKdGain;
  pHandle->wIntegralTerm = 0x00000000UL;
  pHandle->wPrevProcessVarError = 0x00000000UL;
}

void FPID_SetKP( FPID_Handle_t * pHandle, float hKpGain )
{
  pHandle->hKpGain = hKpGain;
}

void FPID_SetKI( FPID_Handle_t * pHandle, float hKiGain )
{
  pHandle->hKiGain = hKiGain;
}

float FPID_GetKP( FPID_Handle_t * pHandle )
{
  return ( pHandle->hKpGain );
}

float FPID_GetKI( FPID_Handle_t * pHandle )
{
  return ( pHandle->hKiGain );
}

float FPID_GetDefaultKP( FPID_Handle_t * pHandle )
{
  return ( pHandle->hDefKpGain );
}

float FPID_GetDefaultKI( FPID_Handle_t * pHandle )
{
  return ( pHandle->hDefKiGain );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
void FPID_SetIntegralTerm( FPID_Handle_t * pHandle, float wIntegralTermValue )
{
  pHandle->wIntegralTerm = wIntegralTermValue;

  return;
}

void FPID_SetLowerIntegralTermLimit( FPID_Handle_t * pHandle, float wLowerLimit )
{
  pHandle->wLowerIntegralLimit = wLowerLimit;
}

void FPID_SetUpperIntegralTermLimit( FPID_Handle_t * pHandle, float wUpperLimit )
{
  pHandle->wUpperIntegralLimit = wUpperLimit;
}

void FPID_SetLowerOutputLimit( FPID_Handle_t * pHandle, float hLowerLimit )
{
  pHandle->hLowerOutputLimit = hLowerLimit;
}

void FPID_SetUpperOutputLimit( FPID_Handle_t * pHandle, float hUpperLimit )
{
  pHandle->hUpperOutputLimit = hUpperLimit;
}

void FPID_SetPrevError( FPID_Handle_t * pHandle, float wPrevProcessVarError )
{
  pHandle->wPrevProcessVarError = wPrevProcessVarError;
  return;
}

void FPID_SetKD( FPID_Handle_t * pHandle, float hKdGain )
{
  pHandle->hKdGain = hKdGain;
}

float FPID_GetKD( FPID_Handle_t * pHandle )
{
  return pHandle->hKdGain;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
float FPI_Controller( FPID_Handle_t * pHandle, float wProcessVarError )
{
  float wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  float wDischarge = 0;
  float hUpperOutputLimit = pHandle->hUpperOutputLimit;
  float hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* Proportional term computation*/
  wProportional_Term = pHandle->hKpGain * wProcessVarError;

  /* Integral term computation */
  if ( FEQ(pHandle->hKiGain, 0.0f) )
  {
    pHandle->wIntegralTerm = 0.0f;
  }
  else
  {
    wIntegral_Term = pHandle->hKiGain * wProcessVarError;
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if ( wIntegral_sum_temp < 0.0f )
    {
      if ( pHandle->wIntegralTerm > 0.0f )
      {
        if ( wIntegral_Term > 0.0f )
        {
          wIntegral_sum_temp = FLOAT_MAX;
        }
      }
    }
    else
    {
      if ( pHandle->wIntegralTerm < 0 )
      {
        if ( wIntegral_Term < 0 )
        {
          wIntegral_sum_temp = -FLOAT_MAX;
        }
      }
    }

    if ( wIntegral_sum_temp > pHandle->wUpperIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    }
    else if ( wIntegral_sum_temp < pHandle->wLowerIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

  wOutput_32 = wProportional_Term + pHandle->wIntegralTerm;

  if ( wOutput_32 > hUpperOutputLimit )
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if ( wOutput_32 < hLowerOutputLimit )
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  }
  else { /* Nothing to do here */ }

  pHandle->wIntegralTerm += wDischarge;

  return ( ( float )( wOutput_32 ) );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
float FPID_Controller( FPID_Handle_t * pHandle, float wProcessVarError )
{
  float wDifferential_Term;
  float wDeltaError;
  float wTemp_output;

  if ( pHandle->hKdGain != 0 ) /* derivative terms not used */
  {
    wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
    wDifferential_Term = pHandle->hKdGain * wDeltaError;

    pHandle->wPrevProcessVarError = wProcessVarError;

    wTemp_output = FPI_Controller( pHandle, wProcessVarError ) + wDifferential_Term;

    if ( wTemp_output > pHandle->hUpperOutputLimit )
    {
      wTemp_output = pHandle->hUpperOutputLimit;
    }
    else if ( wTemp_output < pHandle->hLowerOutputLimit )
    {
      wTemp_output = pHandle->hLowerOutputLimit;
    }
    else
    {}
  }
  else
  {
    wTemp_output = FPI_Controller( pHandle, wProcessVarError );
  }
  return ( wTemp_output );
}
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
