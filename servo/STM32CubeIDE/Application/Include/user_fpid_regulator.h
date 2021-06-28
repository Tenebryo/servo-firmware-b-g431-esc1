/*
 * user_fpid_regulator.h
 *
 *  Created on: May 18, 2021
 *      Author: Sam Blazes
 */

#ifndef APPLICATION_INCLUDE_USER_FPID_REGULATOR_H_
#define APPLICATION_INCLUDE_USER_FPID_REGULATOR_H_

typedef struct FPID_Handle
{
  float   hDefKpGain;           
  float   hDefKiGain;           
  float   hDefKdGain;           
  float   hKpGain;              
  float   hKiGain;              
  float   hKdGain;              

  float   wUpperIntegralLimit;  
  float   wLowerIntegralLimit;
  float   aIntegratorDecay;
  
  float   hUpperOutputLimit;    
  float   hLowerOutputLimit;  
  
  float   wIntegralTerm;        
  float   wPrevProcessVarError; 
                                
} FPID_Handle_t;

/*
 * It initializes the handle
 */
void FPID_HandleInit( FPID_Handle_t * pHandle );

///It updates the Kp gain
void FPID_SetKP( FPID_Handle_t * pHandle, float hKpGain );

/// It updates the Ki gain
void FPID_SetKI( FPID_Handle_t * pHandle, float hKiGain );

/// It returns the Kp gain
float FPID_GetKP( FPID_Handle_t * pHandle );

/// It returns the Ki gain
float FPID_GetKI( FPID_Handle_t * pHandle );

/// It returns the Default Kp gain
float FPID_GetDefaultKP( FPID_Handle_t * pHandle );

/// It returns the Default Ki gain of the passed PI object
float FPID_GetDefaultKI( FPID_Handle_t * pHandle );

/// It set a new value into the PI integral term
void FPID_SetIntegralTerm( FPID_Handle_t * pHandle, float wIntegralTermValue );

/// It set a new value for lower integral term limit
void FPID_SetLowerIntegralTermLimit( FPID_Handle_t * pHandle, float wLowerLimit );

/// It set a new value for upper integral term limit
void FPID_SetUpperIntegralTermLimit( FPID_Handle_t * pHandle, float wUpperLimit );

/// It set a new value for lower output limit
void FPID_SetLowerOutputLimit( FPID_Handle_t * pHandle, float hLowerLimit );

/// It set a new value for upper output limit
void FPID_SetUpperOutputLimit( FPID_Handle_t * pHandle, float hUpperLimit );

/// It set a new value into the FPID Previous error variable required to
/// compute derivative term
void FPID_SetPrevError( FPID_Handle_t * pHandle, float wPrevProcessVarError );

/// @brief  It updates the Kd gain
void FPID_SetKD( FPID_Handle_t * pHandle, float hKdGain );

/// It returns the Kd gain
float FPID_GetKD( FPID_Handle_t * pHandle );

/// This function compute the output of a PI regulator sum of its
/// proportional and integralterms
float FPI_Controller( FPID_Handle_t * pHandle, float wProcessVarError, float dt  );

/// This function compute the output of a FPID regulator sum of its
///  proportional, integral and derivative terms
float FPID_Controller( FPID_Handle_t * pHandle, float wProcessVarError, float dt );

#endif /* APPLICATION_INCLUDE_USER_FPID_REGULATOR_H_ */
