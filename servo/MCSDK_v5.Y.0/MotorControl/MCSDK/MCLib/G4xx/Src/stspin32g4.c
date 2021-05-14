/**
******************************************************************************
* @file    stspin32g4.c
* @author  P. Lombardi, IPC Application Team
* @brief   Implementation of STSPIN32G4 driver library
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
**/

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_def.h>
#include <stm32g4xx_hal_i2c.h>

#include <stdint.h>
#include <stdbool.h>

#include <stspin32g4.h>

#define STSPIN32G4_I2C_TIMEOUT			(100)
#define STSPIN32G4_I2C_LOCKCODE			(0xD)

#define GD_READY_GPIO_Port GPIOE
#define GD_READY_Pin GPIO_PIN_14
#define GD_NFAULT_GPIO_Port GPIOE	
#define GD_NFAULT_Pin GPIO_PIN_15	
#define GD_WAKE_GPIO_Port GPIOE
#define GD_WAKE_Pin	GPIO_PIN_7

static uint8_t STSPIN32G4_bkupREADY;
extern I2C_HandleTypeDef hi2c3;

void SystemClock_Config(void);

HAL_StatusTypeDef STSPIN32G4_init(STSPIN32G4_HandleTypeDef* hdl)
{
  HAL_StatusTypeDef status = HAL_OK;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  hdl->i2cHdl = &hi2c3;

  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GD_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GD_WAKE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GD_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GD_NFAULT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GD_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GD_NFAULT_GPIO_Port, &GPIO_InitStruct);

  if(status != HAL_OK)
    return status;
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_deInit(STSPIN32G4_HandleTypeDef* hdl)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_DeInit(hdl->i2cHdl);

  HAL_GPIO_DeInit(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
  HAL_GPIO_DeInit(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin);
  HAL_GPIO_DeInit(GD_READY_GPIO_Port, GD_READY_Pin);

  hdl->i2cHdl = NULL;
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_lockReg(STSPIN32G4_HandleTypeDef* hdl)
{
  HAL_StatusTypeDef status;
  STSPIN32G4_statusTypeDef statusReg;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  i2cReg = ((STSPIN32G4_I2C_LOCKCODE<<4)&0xf0)|(STSPIN32G4_I2C_LOCKCODE&0x0f);
  status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOCK, i2cReg);
  
#ifdef STSPIN32G4_I2C_LOCKUSEPARANOID
  if(status == HAL_OK)
    status = STSPIN32G4_getStatus(hdl, &statusReg);
  
  if(status == HAL_OK)
    if(statusReg.lock != 1)
      status = HAL_ERROR;
#endif
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_unlockReg(STSPIN32G4_HandleTypeDef* hdl)
{
  HAL_StatusTypeDef status;
  STSPIN32G4_statusTypeDef statusReg;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  i2cReg = (((~STSPIN32G4_I2C_LOCKCODE)<<4)&0xf0)|(STSPIN32G4_I2C_LOCKCODE&0x0f);
  status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOCK, i2cReg);
  
#ifdef STSPIN32G4_I2C_LOCKUSEPARANOID
  if(status == HAL_OK)
    status = STSPIN32G4_getStatus(hdl, &statusReg);
  
  if(status == HAL_OK)
    if(statusReg.lock == 1)
      status = HAL_ERROR;
#endif
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_readReg(STSPIN32G4_HandleTypeDef* hdl, uint8_t regAddr, uint8_t* value)
{
  HAL_StatusTypeDef status;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  if(value == NULL)
    return HAL_ERROR;
  
  status = HAL_I2C_Mem_Read(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, (uint16_t) regAddr, 1, value, 1, STSPIN32G4_I2C_TIMEOUT);
  return status;
}

HAL_StatusTypeDef STSPIN32G4_writeReg(STSPIN32G4_HandleTypeDef* hdl, uint8_t regAddr, uint8_t value)
{
  HAL_StatusTypeDef status;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  status = HAL_I2C_Mem_Write(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, (uint16_t) regAddr, 1, &value, 1, STSPIN32G4_I2C_TIMEOUT);
  return status;
}

HAL_StatusTypeDef STSPIN32G4_writeVerifyReg(STSPIN32G4_HandleTypeDef* hdl, uint8_t regAddr, uint8_t value)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  status = STSPIN32G4_writeReg(hdl, regAddr, value);
  
  if(status == HAL_OK)
    status = STSPIN32G4_readReg(hdl, regAddr, &i2cReg);
  
  if(status == HAL_OK)
    if(value != i2cReg)
      status = HAL_ERROR;
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_set3V3(STSPIN32G4_HandleTypeDef* hdl, bool enabled)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
  {
    if(enabled)
      i2cReg &= ~STSPIN32G4_I2C_REG3V3_DIS;
    else
      i2cReg |= STSPIN32G4_I2C_REG3V3_DIS;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_POWMNG, i2cReg);
  }
  
  if(status == HAL_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

HAL_StatusTypeDef STSPIN32G4_get3V3(STSPIN32G4_HandleTypeDef* hdl, bool* enabled)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  if(enabled == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);
  *enabled = (i2cReg&STSPIN32G4_I2C_REG3V3_DIS) ? false : true;
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_setVCC(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_confVCC vcc)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
  {
    switch(vcc.voltage)
    {
    case _EXT:
      i2cReg |= STSPIN32G4_I2C_VCC_DIS;
      break;
      
    case _8V:
      i2cReg &= ~(STSPIN32G4_I2C_VCC_DIS|STSPIN32G4_I2C_VCC_VAL_3);
      i2cReg |= STSPIN32G4_I2C_VCC_VAL_0;
      break;
      
    case _10V:
      i2cReg &= ~(STSPIN32G4_I2C_VCC_DIS|STSPIN32G4_I2C_VCC_VAL_3);
      i2cReg |= STSPIN32G4_I2C_VCC_VAL_1;
      break;
      
    case _12V:
      i2cReg &= ~(STSPIN32G4_I2C_VCC_DIS|STSPIN32G4_I2C_VCC_VAL_3);
      i2cReg |= STSPIN32G4_I2C_VCC_VAL_2;
      break;
      
    case _15V:
      i2cReg &= ~STSPIN32G4_I2C_VCC_DIS;
      i2cReg |= STSPIN32G4_I2C_VCC_VAL_3;
      break;
    default:
      status = HAL_ERROR;
      break;
    }
  }
  
  if(status == HAL_OK)
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_POWMNG, i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  
  if(status == HAL_OK) // configuration of nFAULT pin
  {
    if(vcc.useNFAULT)
      i2cReg |= STSPIN32G4_I2C_VCC_UVLO_FLT;
    else
      i2cReg &= ~STSPIN32G4_I2C_VCC_UVLO_FLT;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_NFAULT, i2cReg);
  }
  
  if(status == HAL_OK)
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
  
  if(status == HAL_OK) // configuration of READY pin
  {
    if(vcc.useREADY)
      i2cReg |= STSPIN32G4_I2C_VCC_UVLO_RDY;
    else
      i2cReg &= ~STSPIN32G4_I2C_VCC_UVLO_RDY;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, i2cReg);
  }
  
  if(status == HAL_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

HAL_StatusTypeDef STSPIN32G4_getVCC(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_confVCC* vcc)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  if(vcc == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);
  
  if(status == HAL_OK)
  {
    if(i2cReg&STSPIN32G4_I2C_VCC_DIS)
    {
      vcc->voltage = _EXT;
    }
    else
    {
      switch(i2cReg&STSPIN32G4_I2C_VCC_VAL_3)
      {
      case STSPIN32G4_I2C_VCC_VAL_0:
        vcc->voltage = _8V;
        break;
        
      case STSPIN32G4_I2C_VCC_VAL_1:
        vcc->voltage = _10V;
        break;
        
      case STSPIN32G4_I2C_VCC_VAL_2:
        vcc->voltage = _12V;
        break;
        
      case STSPIN32G4_I2C_VCC_VAL_3:
        vcc->voltage = _15V;
        break;
        
      default:
        status = HAL_ERROR;
        break;
      }
    }
  }
  
  if(status == HAL_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
    vcc->useNFAULT = (i2cReg&STSPIN32G4_I2C_VCC_UVLO_FLT) ? true : false;
  }
  
  if(status == HAL_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
    vcc->useREADY = (i2cReg&STSPIN32G4_I2C_VCC_UVLO_RDY) ? true : false;
  }
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_setInterlocking(STSPIN32G4_HandleTypeDef* hdl, bool enabled)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
  {
    if(enabled)
      i2cReg |= STSPIN32G4_I2C_ILOCK;
    else
      i2cReg &= ~STSPIN32G4_I2C_ILOCK;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOGIC, i2cReg);
  }
  
  if(status == HAL_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

HAL_StatusTypeDef STSPIN32G4_getInterlocking(STSPIN32G4_HandleTypeDef* hdl, bool* enabled)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  if(enabled == NULL)
    return HAL_ERROR;	
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  *enabled = (i2cReg&STSPIN32G4_I2C_ILOCK) ? true : false;
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_setMinimumDeadTime(STSPIN32G4_HandleTypeDef* hdl, bool enabled)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
  {
    if(enabled)
      i2cReg |= STSPIN32G4_I2C_DTMIN;
    else
      i2cReg &= ~STSPIN32G4_I2C_DTMIN;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOGIC, i2cReg);
  }
  
  if(status == HAL_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

HAL_StatusTypeDef STSPIN32G4_getMinimumDeadTime(STSPIN32G4_HandleTypeDef* hdl, bool* enabled)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  if(enabled == NULL)
    return HAL_ERROR;	
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  *enabled = (i2cReg&STSPIN32G4_I2C_DTMIN) ? true : false;
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_setVDSP(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_confVDSP vdsp)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
  {
    i2cReg &= ~STSPIN32G4_I2C_VDS_P_DEG_3;
    switch(vdsp.deglitchTime)
    {
    case _6us:
      i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_0;
      break;
      
    case _4us:
      i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_1;
      break;
      
    case _3us:
      i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_2;
      break;
      
    case _2us:
      i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_3;
      break;
      
    default:
      status = HAL_ERROR;
      break;
    }
  }
  if(status == HAL_OK)
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOGIC, i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  
  if(status == HAL_OK) // configure nFault signaling
  {
    if(vdsp.useNFAULT)
      i2cReg |= STSPIN32G4_I2C_VDS_P_FLT;
    else
      i2cReg &= ~STSPIN32G4_I2C_VDS_P_FLT;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_NFAULT, i2cReg);
  }
  
  if(status == HAL_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

HAL_StatusTypeDef STSPIN32G4_getVDSP(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_confVDSP* vdsp)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  if(vdsp == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  
  if(status == HAL_OK)
  {
    switch(i2cReg&STSPIN32G4_I2C_VDS_P_DEG_3)
    {
    case STSPIN32G4_I2C_VDS_P_DEG_0:
      vdsp->deglitchTime = _6us;
      break;
      
    case STSPIN32G4_I2C_VDS_P_DEG_1:
      vdsp->deglitchTime = _4us;
      break;
      
    case STSPIN32G4_I2C_VDS_P_DEG_2:
      vdsp->deglitchTime = _3us;
      break;			
      
    case STSPIN32G4_I2C_VDS_P_DEG_3:
      vdsp->deglitchTime = _2us;
      break;
      
    default:
      status = HAL_ERROR;
      break;
    }
  }
  
  if(status == HAL_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
    vdsp->useNFAULT = (i2cReg&STSPIN32G4_I2C_VDS_P_FLT) ? true : false;
  }
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_setTHSD(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_confTHSD thsd)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
  {
    if(thsd.useNFAULT)
      i2cReg |= STSPIN32G4_I2C_THSD_FLT;
    else
      i2cReg &= ~STSPIN32G4_I2C_THSD_FLT;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_NFAULT, i2cReg);
  }
  
  if(status == HAL_OK)
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
  
  if(status == HAL_OK)
  {
    if(thsd.useREADY)
      i2cReg |= STSPIN32G4_I2C_THSD_RDY;
    else
      i2cReg &= ~STSPIN32G4_I2C_THSD_RDY;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, i2cReg);
  }
  
  if(status == HAL_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

HAL_StatusTypeDef STSPIN32G4_getTHSD(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_confTHSD* thsd)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  
  if(hdl == NULL)
    return HAL_ERROR;	
  
  if(thsd == NULL)
    return HAL_ERROR; 
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  thsd->useNFAULT = (i2cReg&STSPIN32G4_I2C_THSD_FLT) ? true : false;
  
  if(status == HAL_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
    thsd->useREADY = (i2cReg&STSPIN32G4_I2C_THSD_RDY) ? true : false;
  }
  
  return status;
  
} 

HAL_StatusTypeDef STSPIN32G4_clearFaults(STSPIN32G4_HandleTypeDef* hdl)
{
  uint8_t i2cReg = 0xff;	
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  return STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_CLEAR, i2cReg);
}

HAL_StatusTypeDef STSPIN32G4_reset(STSPIN32G4_HandleTypeDef* hdl)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0xff;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK)
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_RESET, i2cReg);
  
  if(status == HAL_OK)
    STSPIN32G4_lockReg(hdl);
  
  return status;
}

HAL_StatusTypeDef STSPIN32G4_standby(STSPIN32G4_HandleTypeDef* hdl, bool enableStbyReg)
{
  HAL_StatusTypeDef status;
  uint8_t i2cReg = 0;
  uint32_t tickStart;
  uint32_t tickFreq = HAL_GetTickFreq()/1000; // in kHz to have ms base

  if(hdl == NULL)
    return HAL_ERROR;
  
  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);
  
  if(status == HAL_OK)
    status = STSPIN32G4_unlockReg(hdl);
  
  if(status == HAL_OK) // configure the Standby regulator
  {
    if(enableStbyReg)
      i2cReg |= STSPIN32G4_I2C_STBY_REG_EN;
    else
      i2cReg &= ~STSPIN32G4_I2C_STBY_REG_EN;
    
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_POWMNG, i2cReg);
  }
  
  if(status == HAL_OK) // create backup of the READY register
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &STSPIN32G4_bkupREADY);
  
  if(status == HAL_OK) // set only STBY_RDY to signal the exit from standby
  {
    i2cReg = STSPIN32G4_I2C_STBY_RDY;
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, i2cReg);
  }
  
  if(status == HAL_OK) // WAKE line low
    HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_RESET);
  
#ifdef STSPIN32G4_HSI16
  if(status == HAL_OK)
    if(enableStbyReg)
    {
      status = HAL_RCC_DeInit(); // HSI16 to reduce current consumption
    }
#endif
  
  if(status == HAL_OK) // request to enter standby
  {
	  i2cReg = 0x01;
	  status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_STBY, i2cReg);
  }

  if(HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_SET)
  {
	  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);
	  status =  HAL_ERROR;
  }
  else
  {
	  tickStart = HAL_GetTick();
	  while(HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_RESET)
	  {
		  if((HAL_GetTick() - tickStart) > (1*tickFreq)) // expected time is 100us
		  {
			  status = HAL_TIMEOUT;
			  break;
		  }
	  }
  }

  // wait 1ms and check possible exit from standby
  HAL_Delay(1);
  if ((HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) != GPIO_PIN_SET) ||
		  (HAL_GPIO_ReadPin(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin) != GPIO_PIN_SET))
	  status = HAL_ERROR;

#ifdef STSPIN32G4_HSI16
  // if the driver failed to enter standby clock is reconfigured
  if(status != HAL_OK)
	  SystemClock_Config(); // restore clock configuration
#endif

  return status;
}

HAL_StatusTypeDef STSPIN32G4_wakeup(STSPIN32G4_HandleTypeDef* hdl, uint8_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t tickStart;
  uint32_t tickFreq = HAL_GetTickFreq()/1000; // in kHz to have ms base
  STSPIN32G4_statusTypeDef statReg;
  
  if(hdl == NULL)
    return HAL_ERROR;
  
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);
  
  if(timeout_ms < 4)
    timeout_ms = 4;	// The soft start is expected to take 4ms
  
  tickStart = HAL_GetTick();
  
  while(HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_RESET)
    if((HAL_GetTick() - tickStart) > (timeout_ms*tickFreq))	
    {
      status = HAL_TIMEOUT;
      break;
    }
  
#ifdef STSPIN32G4_HSI16
  if(status == HAL_OK)
    SystemClock_Config();
#endif
  
  // Restore READY register
  if(status == HAL_OK)
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, STSPIN32G4_bkupREADY);
  
  if(status == HAL_OK)
    status = STSPIN32G4_lockReg(hdl);

  if(status == HAL_OK)
    status = STSPIN32G4_getStatus(hdl, &statReg);

  if(status == HAL_OK)
    if(statReg.reset == 1)
       status = HAL_ERROR;

  return status;
}

HAL_StatusTypeDef STSPIN32G4_getStatus(STSPIN32G4_HandleTypeDef* hdl, STSPIN32G4_statusTypeDef* status)
{
  if(hdl == NULL)
    return HAL_ERROR;
  
  if(status == NULL)
    return HAL_ERROR;
  
  return STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_STATUS, (uint8_t*)status);
}

