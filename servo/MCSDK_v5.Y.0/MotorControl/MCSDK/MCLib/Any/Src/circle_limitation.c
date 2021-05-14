/**
  ******************************************************************************
  * @file    circle_limitation.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the functions that implement the circle
  *          limitation feature of the STM32 Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "circle_limitation.h"
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup CircleLimitation Circle Limitation
  * @brief Circle Limitation component of the Motor Control SDK
  *
  * @todo Document the Circle Limitation "module".
  *
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

#if defined CIRCLE_LIMITATION_SQRT_M0
const uint16_t SqrtTable[1025] = SQRT_CIRCLE_LIMITATION;
#endif
__weak qd_t Circle_Limitation(CircleLimitation_Handle_t * pHandle, qd_t Vqd)
{
  int32_t MaxModule;
  int32_t square_q;
  int32_t square_temp;
  int32_t square_d;
  int32_t square_sum;
  int32_t square_limit;
  int32_t vd_square_limit;
  int32_t new_q;
  int32_t new_d;
  qd_t Local_Vqd=Vqd;

  MaxModule = pHandle->MaxModule;

  square_q = (int32_t)(Vqd.q) * Vqd.q;
  square_d = (int32_t)(Vqd.d) * Vqd.d;
  square_limit = MaxModule * MaxModule;
  vd_square_limit = pHandle->MaxVd * pHandle->MaxVd;
  square_sum = square_q + square_d;

  if (square_sum > square_limit)
  {
    if(square_d <= vd_square_limit)
    {
#if defined CIRCLE_LIMITATION_SQRT_M0
      square_temp = (square_limit - square_d)/1048576;
      new_q = SqrtTable[square_temp];
#else
      square_temp = square_limit - square_d;
      new_q = MCM_Sqrt(square_temp);
#endif
      if(Vqd.q < 0)
      {
        new_q = -new_q;
      }
      new_d = Vqd.d;
    }
    else
    {
      new_d = pHandle->MaxVd;
      if(Vqd.d < 0)
      {
        new_d = -new_d;
      }
#if defined CIRCLE_LIMITATION_SQRT_M0
      square_temp = (square_limit - vd_square_limit)/1048576;
      new_q = SqrtTable[square_temp];
#else
      square_temp = square_limit - vd_square_limit;
      new_q = MCM_Sqrt(square_temp);
#endif
      if(Vqd.q < 0)
      {
        new_q = - new_q;
      }
    }
    Local_Vqd.q = new_q;
    Local_Vqd.d = new_d;
  }
  return(Local_Vqd);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

