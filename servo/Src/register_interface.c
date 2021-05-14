
#include "stdint.h"
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mcp.h"
#include "mcp_config.h"
#include "mcpa.h"
#include "mc_configuration_registers.h"

BusVoltageSensor_Handle_t* BusVoltageSensor[NBR_OF_MOTORS]={ &BusVoltageSensor_M1._Super};
SpeednPosFdbk_Handle_t * SpeedSensor [NBR_OF_MOTORS] = {(SpeednPosFdbk_Handle_t *) &ENCODER_M1 };
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS] = { &PIDSpeedHandle_M1 };
ENCODER_Handle_t *pEncoder[NBR_OF_MOTORS] = {&ENCODER_M1};

static uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t maxSize);
static uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t maxSize);
static uint8_t RI_MovString (const char * srcString, char * destString, uint16_t *size, uint16_t maxSize);

__weak uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle)
{
  uint16_t * dataElementID;
  uint8_t * rxData = pHandle->rxBuffer+MCP_HEADER_SIZE;
  uint8_t * txData = pHandle->txBuffer;
  int16_t rxLength = pHandle->rxLength-MCP_HEADER_SIZE;
  int16_t txSyncFreeSpace = pHandle->pTransportLayer->txSyncMaxPayload-1; /* 1 Byte must be available for MCP CMD status*/
  uint16_t size;
  uint8_t retVal=MCP_CMD_OK;
  uint8_t accessResult;
  uint8_t number_of_item =0;
  pHandle->txLength = 0;
  while (rxLength > 0)
  {
     number_of_item ++;
     dataElementID = (uint16_t *) rxData;
     rxLength = rxLength-MCP_ID_SIZE; // We consume 2 byte in the DataID
     rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next data
     accessResult = RI_SetReg (*dataElementID,rxData,&size,rxLength);

     /* Prepare next data*/
     rxLength = (int16_t) (rxLength - size);
     rxData = rxData+size;
     /* If there is only one CMD in the buffer, we do not store the result */
     if (number_of_item == 1 && rxLength == 0)
     {
       retVal = accessResult;
     }
     else
     {/* Store the result for each access to be able to report failling access */
       if (txSyncFreeSpace >=0 )
       {
         *txData = accessResult;
         txData = txData+1;
         pHandle->txLength++;
         txSyncFreeSpace--;
         retVal = (accessResult != MCP_CMD_OK) ? MCP_CMD_NOK : retVal;
         if ((accessResult == MCP_ERROR_BAD_DATA_TYPE) || (accessResult == MCP_ERROR_BAD_RAW_FORMAT))
         { /* From this point we are not able to continue to decode CMD buffer*/
           /* We stop the parsing */
           rxLength = 0;
         }
       }
       else
       {
         /* Stop parsing the cmd buffer as no space to answer */
         /* If we reach this state, chances are high the command was badly formated or received */
         rxLength = 0;
         retVal = MCP_ERROR_NO_TXSYNC_SPACE;
       }
     }
  }
  /* If all accesses are fine, just one global MCP_CMD_OK is required*/
  if (retVal == MCP_CMD_OK)
  {
    pHandle->txLength = 0;
  }
  return retVal;
}

__weak uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle)
{
  uint16_t * dataElementID;
  uint8_t * rxData = pHandle->rxBuffer+MCP_HEADER_SIZE;
  uint8_t * txData = pHandle->txBuffer;
  uint16_t size = 0;
  uint16_t rxLength = pHandle->rxLength-MCP_HEADER_SIZE;
  uint16_t txSyncFreeSpace = pHandle->pTransportLayer->txSyncMaxPayload-1; /* 1 Byte must be available for MCP CMD status*/
  uint8_t retVal = MCP_CMD_NOK;
  pHandle->txLength = 0;
  while (rxLength > 0)
  {
     dataElementID = (uint16_t *) rxData;
     rxLength = rxLength-MCP_ID_SIZE;
     rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next MCP_ID
     retVal = RI_GetReg (*dataElementID,txData, &size, txSyncFreeSpace);
     if (retVal == MCP_CMD_OK )
     {
       txData = txData+size;
       pHandle->txLength += size;
       txSyncFreeSpace = txSyncFreeSpace-size;
     }
     else
     {
       rxLength = 0;
     }
  }
  return retVal;
}

uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t freeSpace)
{
  uint8_t typeID;
  uint8_t motorID;
  uint8_t retVal = MCP_CMD_OK;
  uint16_t regID = dataID & REG_MASK;
  typeID = dataID & TYPE_MASK;
  motorID = (dataID & MOTOR_MASK)-1;

  MCI_Handle_t * pMCI = &Mci[motorID];

  switch (typeID)
  {
  case TYPE_DATA_8BIT:
    {
    uint8_t regdata8 = *data;

    switch (regID)
    {
    case MC_REG_STATUS:
      retVal = MCP_ERROR_RO_REG;
      break;
    case MC_REG_CONTROL_MODE:
      {
        if ((STC_Modality_t)regdata8 == STC_TORQUE_MODE)
        {
          MCI_ExecTorqueRamp(pMCI, MCI_GetTeref(pMCI),0);
        }
        if ((STC_Modality_t)regdata8 == STC_SPEED_MODE)
        {
          MCI_ExecSpeedRamp(pMCI, MCI_GetMecSpeedRefUnit(pMCI),0);
        }
      }
      break;
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;
    }
    *size = 1;
    break;
    }
  case TYPE_DATA_16BIT:
    {
    uint16_t regdata16 = *(uint16_t *)data;
    switch (regID)
    {
    case MC_REG_SPEED_KP:
      PID_SetKP(pPIDSpeed[motorID], regdata16);
      break;
    case MC_REG_SPEED_KI:
      PID_SetKI(pPIDSpeed[motorID], regdata16);
      break;
    case MC_REG_SPEED_KD:
      PID_SetKD(pPIDSpeed[motorID], regdata16);
     break;
    case MC_REG_I_Q_KP:
      PID_SetKP(pPIDIq[motorID], regdata16);
     break;
    case MC_REG_I_Q_KI:
      PID_SetKI(pPIDIq[motorID], regdata16);
     break;
    case MC_REG_I_Q_KD:
      PID_SetKD(pPIDIq[motorID], regdata16);
     break;
    case MC_REG_I_D_KP:
      PID_SetKP(pPIDId[motorID], regdata16);
     break;
    case MC_REG_I_D_KI:
      PID_SetKI(pPIDId[motorID], regdata16);
     break;
    case MC_REG_I_D_KD:
      PID_SetKD(pPIDId[motorID], regdata16);
     break;

    case MC_REG_BUS_VOLTAGE:
    case MC_REG_HEATS_TEMP:
    case MC_REG_MOTOR_POWER:
      retVal = MCP_ERROR_RO_REG;
     break;
    case MC_REG_I_A:
    case MC_REG_I_B:
    case MC_REG_I_ALPHA:
    case MC_REG_I_BETA:
    case MC_REG_I_Q:
    case MC_REG_I_D:
    case MC_REG_FLUXWK_BUS_MEAS:
      retVal = MCP_ERROR_RO_REG;
      break;
    case MC_REG_I_Q_REF:
      {
        qd_t currComp;
        currComp = MCI_GetIqdref(pMCI);
        currComp.q = (int16_t)regdata16;
        MCI_SetCurrentReferences(pMCI,currComp);
      }
      break;
    case MC_REG_I_D_REF:
      {
        qd_t currComp;
        currComp = MCI_GetIqdref(pMCI);
        currComp.d = regdata16;
        MCI_SetCurrentReferences(pMCI,currComp);
      }
      break;
    case MC_REG_V_Q:
    case MC_REG_V_D:
    case MC_REG_V_ALPHA:
    case MC_REG_V_BETA:
        case MC_REG_ENCODER_EL_ANGLE:
    case MC_REG_ENCODER_SPEED:
    case MC_REG_HALL_EL_ANGLE:
    case MC_REG_HALL_SPEED:
      retVal = MCP_ERROR_RO_REG;
     break;
    case MC_REG_DAC_USER1:
     break;
    case MC_REG_DAC_USER2:
     break;
    case MC_REG_SPEED_KP_DIV:
      PID_SetKPDivisorPOW2(pPIDSpeed[motorID], regdata16);
     break;
    case MC_REG_SPEED_KI_DIV:
      PID_SetKDDivisorPOW2(pPIDSpeed[motorID], regdata16);
     break;
    case MC_REG_SPEED_KD_DIV:
      PID_SetKDDivisorPOW2(pPIDSpeed[motorID], regdata16);
     break;
    case MC_REG_I_D_KP_DIV:
      PID_SetKPDivisorPOW2(pPIDId[motorID], regdata16);
      break;
    case MC_REG_I_D_KI_DIV:
      PID_SetKIDivisorPOW2(pPIDId[motorID], regdata16);
      break;
    case MC_REG_I_D_KD_DIV:
      PID_SetKDDivisorPOW2(pPIDId[motorID], regdata16);
      break;
    case MC_REG_I_Q_KP_DIV:
      PID_SetKPDivisorPOW2(pPIDIq[motorID], regdata16);
      break;
    case MC_REG_I_Q_KI_DIV:
      PID_SetKIDivisorPOW2(pPIDIq[motorID], regdata16);
      break;
    case MC_REG_I_Q_KD_DIV:
      PID_SetKDDivisorPOW2(pPIDIq[motorID], regdata16);
      break;

    default:
      retVal = MCP_ERROR_UNKNOWN_REG;

    }
    *size = 2;
    }

    break;

  case TYPE_DATA_32BIT:
    {
    int32_t regdata32 = *(int32_t *)data;
    switch (regID)
    {
    case MC_REG_FAULTS_FLAGS:
    case MC_REG_SPEED_MEAS:
      retVal = MCP_ERROR_RO_REG;
      break;
    case MC_REG_SPEED_REF:
      MCI_ExecSpeedRamp(pMCI,(int16_t)((regdata32*SPEED_UNIT)/_RPM),0);
      break;
    case MC_REG_STOPLL_EST_BEMF:
    case MC_REG_STOPLL_OBS_BEMF:
    case MC_REG_STOCORDIC_EST_BEMF:
    case MC_REG_STOCORDIC_OBS_BEMF:
      retVal = MCP_ERROR_RO_REG;
      break;
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;
    }
    *size = 4;
    break;
    }
  case TYPE_DATA_STRING:
    {
      const char *charData = (const char *) data;
      char *dummy = (char *) data ;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist*/
      /* It allows to jump to the next command in the buffer */
      RI_MovString (charData, dummy, size, freeSpace);
    }
    break;
  case TYPE_DATA_RAW:
    {
      uint16_t rawSize = *(uint16_t *) data;
      *size = rawSize+2; /* The size consumed by the structure is the structure size + 2 bytes used to store the size*/
      uint8_t * rawData = data+2; /* rawData points to the first data (after size extraction) */
      if (*size > freeSpace )
      { /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer construction*/
        *size =0;
        retVal = MCP_ERROR_BAD_RAW_FORMAT; /* this error stop the parsing of the CMD buffer */
      }
      else
      {
        switch (regID)
        {
        case MC_REG_GLOBAL_CFG:
        case MC_REG_MOTOR_CFG:
        case MC_REG_FOCFW_CFG:
          retVal = MCP_ERROR_RO_REG;
          break;
        case MC_REG_SPEED_RAMP:
          {
            int32_t rpm;
            uint16_t duration;
            rpm = *(int32_t *)rawData;
            duration = *(uint16_t *)&rawData[4];
            MCI_ExecSpeedRamp(pMCI,(int16_t)((rpm*SPEED_UNIT)/_RPM),duration);
          }
          break;
        case MC_REG_TORQUE_RAMP:
          {
            uint32_t torque;
            uint16_t duration;
            torque = *(int32_t *)rawData;
            duration = *(uint16_t *)&rawData[4];
            MCI_ExecTorqueRamp(pMCI,torque,duration);
          }
          break;

        case MC_REG_ASYNC_UARTA:
          {
           retVal =  MCPA_cfgLog ( &MCPA_UART_A, rawData );
          }
          break;
        case MC_REG_CURRENT_REF:
          {
            qd_t currComp;
            currComp.q = *((uint16_t *) rawData);
            currComp.d = *((uint16_t *) &rawData[2]);
            MCI_SetCurrentReferences(pMCI,currComp);
          }
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
      }
    }
    break;
  default:
    retVal = MCP_ERROR_BAD_DATA_TYPE;
    *size =0; /* From this point we are not able anymore to decode the RX buffer*/
  }
  return retVal;
}

uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t freeSpace )
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t motorID = (dataID & MOTOR_MASK)-1;
  uint16_t regID = dataID & REG_MASK;
  uint8_t retVal = MCP_CMD_OK;

  MCI_Handle_t * pMCI = &Mci[motorID];
  switch (typeID)
  {
  case TYPE_DATA_8BIT:
    {
      if (freeSpace > 0 )
      {
        switch (regID)
        {
        case MC_REG_STATUS:
          *data = STM_GetState(&STM[motorID]);
          break;
        case MC_REG_CONTROL_MODE:
          *data =  MCI_GetControlMode(pMCI);
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 1;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_16BIT:
    {
      uint16_t * regdataU16 = (uint16_t *)data;
      int16_t * regdata16 = (int16_t *) data;
      if (freeSpace >= 2 )
      {
        switch (regID)
        {
        case MC_REG_SPEED_KP:
          *regdata16 = PID_GetKP(pPIDSpeed[motorID]);
          break;
        case MC_REG_SPEED_KI:
          *regdata16 = PID_GetKI(pPIDSpeed[motorID]);
          break;
        case MC_REG_SPEED_KD:
          *regdata16 = PID_GetKD(pPIDSpeed[motorID]);
         break;
        case MC_REG_I_Q_KP:
          *regdata16 = PID_GetKP(pPIDIq[motorID]);
         break;
        case MC_REG_I_Q_KI:
          *regdata16 = PID_GetKI(pPIDIq[motorID]);
         break;
        case MC_REG_I_Q_KD:
          *regdata16 = PID_GetKD(pPIDIq[motorID]);
         break;
        case MC_REG_I_D_KP:
          *regdata16 = PID_GetKP(pPIDId[motorID]);
         break;
        case MC_REG_I_D_KI:
          *regdata16 = PID_GetKI(pPIDId[motorID]);
         break;
        case MC_REG_I_D_KD:
          *regdata16 = PID_GetKD(pPIDId[motorID]);
         break;
        case MC_REG_BUS_VOLTAGE:
         *regdataU16 = VBS_GetAvBusVoltage_V(BusVoltageSensor[motorID]);
         break;
        case MC_REG_HEATS_TEMP:
          *regdata16 = NTC_GetAvTemp_C(pTemperatureSensor[motorID]);
         break;
        case MC_REG_MOTOR_POWER:
          *regdata16 = MPM_GetAvrgElMotorPowerW((MotorPowMeas_Handle_t *)pMPM[motorID]);
         break;
        case MC_REG_I_A:
          *regdata16 = MCI_GetIab(pMCI).a;
         break;
        case MC_REG_I_B:
          *regdata16 = MCI_GetIab(pMCI).b;
         break;
        case MC_REG_I_ALPHA:
          *regdata16 = MCI_GetIalphabeta(pMCI).alpha;
         break;
        case MC_REG_I_BETA:
          *regdata16 = MCI_GetIalphabeta(pMCI).beta;
         break;
        case MC_REG_I_Q:
          *regdata16 = MCI_GetIqd(pMCI).q;
         break;
        case MC_REG_I_D:
          *regdata16 = MCI_GetIqd(pMCI).d;
         break;
        case MC_REG_I_Q_REF:
          *regdata16 = MCI_GetIqdref(pMCI).q;
         break;
        case MC_REG_I_D_REF:
          *regdata16 = MCI_GetIqdref(pMCI).d;
         break;
        case MC_REG_V_Q:
          *regdata16 = MCI_GetVqd(pMCI).q;
         break;
        case MC_REG_V_D:
          *regdata16 = MCI_GetVqd(pMCI).d;
         break;
        case MC_REG_V_ALPHA:
          *regdata16 = MCI_GetValphabeta(pMCI).alpha;
         break;
        case MC_REG_V_BETA:
          *regdata16 = MCI_GetValphabeta(pMCI).beta;
         break;
        case MC_REG_ENCODER_EL_ANGLE:
          *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) pEncoder[motorID]);
         break;
        case MC_REG_ENCODER_SPEED:
          *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) pEncoder[motorID]);
         break;
        case MC_REG_DAC_USER1:
         break;
        case MC_REG_DAC_USER2:
         break;

        case MC_REG_SPEED_KP_DIV:
          *regdataU16 = PID_GetKPDivisor(pPIDSpeed[motorID]);
         break;
        case MC_REG_SPEED_KI_DIV:
          *regdataU16 = PID_GetKIDivisor(pPIDSpeed[motorID]);
         break;
        case MC_REG_SPEED_KD_DIV:
          *regdataU16 = PID_GetKDDivisor(pPIDSpeed[motorID]);
         break;
        case MC_REG_I_D_KP_DIV:
          *regdataU16 = PID_GetKPDivisor(pPIDId[motorID]);
          break;
        case MC_REG_I_D_KI_DIV:
          *regdataU16 = PID_GetKIDivisor(pPIDId[motorID]);
          break;
        case MC_REG_I_D_KD_DIV:
          *regdataU16 = PID_GetKDDivisor(pPIDId[motorID]);
          break;
        case MC_REG_I_Q_KP_DIV:
          *regdataU16 = PID_GetKPDivisor(pPIDIq[motorID]);
          break;
        case MC_REG_I_Q_KI_DIV:
          *regdataU16 = PID_GetKIDivisor(pPIDIq[motorID]);
          break;
        case MC_REG_I_Q_KD_DIV:
          *regdataU16 = PID_GetKDDivisor(pPIDIq[motorID]);
          break;

        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 2;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_32BIT:
    {
      uint32_t *regdataU32 = (uint32_t *) data;
      int32_t *regdata32 = (int32_t *) data;
      if ( freeSpace >= 4)
      {
        switch (regID)
        {
        case MC_REG_FAULTS_FLAGS:
          *regdataU32 = STM_GetFaultState(&STM[motorID]);
          break;
        case MC_REG_SPEED_MEAS:
          *regdata32 = (((int32_t)MCI_GetAvrgMecSpeedUnit(pMCI) * _RPM)/SPEED_UNIT);
          break;
        case MC_REG_SPEED_REF:
          *regdata32 = (((int32_t)MCI_GetMecSpeedRefUnit(pMCI)*_RPM)/SPEED_UNIT);
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 4;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_STRING:
    {
      char *charData = (char *) data;
      switch (regID)
      {
      case MC_REG_FW_VERSION:

        break;
      case MC_REG_CTRL_BOARD_NAME:
        retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
        break;
      case MC_REG_PWR_BOARD_NAME:
        retVal = RI_MovString (PWR_BOARD_NAME[motorID] ,charData, size, freeSpace);
        break;
      case MC_REG_MOTOR_NAME:
        retVal = RI_MovString (MOTOR_NAME[motorID] ,charData, size, freeSpace);
        break;
      default:
        retVal = MCP_ERROR_UNKNOWN_REG;
        *size= 0 ; /* */
      }
    }
    break;
  case TYPE_DATA_RAW:
    {
      uint16_t *rawSize = (uint16_t *) data; /* First 2 bytes of the answer is reserved to the size */
      uint8_t * rawData = data+2;

      switch (regID)
      {
        case MC_REG_GLOBAL_CFG:
        *rawSize = sizeof(GlobalConfig_reg_t);
        if ((*rawSize) +2  > freeSpace)
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        else
        {
          memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t) );
        }
        break;
      case MC_REG_MOTOR_CFG:
        *rawSize = sizeof(MotorConfig_reg_t);
        if ((*rawSize) +2  > freeSpace)
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        else
        {
          memcpy(rawData, MotorConfig_reg[motorID], sizeof(MotorConfig_reg_t) );
        }
      break;
      case MC_REG_FOCFW_CFG:
        *rawSize = sizeof(FOCFwConfig_reg_t);
        if ((*rawSize) +2  > freeSpace)
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        else
        {
          memcpy(rawData, FOCConfig_reg[motorID], sizeof(FOCFwConfig_reg_t) );
        }
      break;
      case MC_REG_SPEED_RAMP:
        {
          int32_t *rpm = (int32_t *) rawData;
          uint16_t *duration = (uint16_t *) &rawData[4];
          *rpm = (int32_t)((MCI_GetLastRampFinalSpeed(pMCI) * _RPM)/SPEED_UNIT) ;
          *duration = MCI_GetLastRampFinalDuration(pMCI) ;
          *rawSize = 6;
        }
        break;
      case MC_REG_TORQUE_RAMP:
        {
          int16_t *torque = (int16_t *) rawData;
          uint16_t *duration = (uint16_t *) &rawData[2];
          *rawSize = 4;
          *torque = MCI_GetLastRampFinalTorque(pMCI);
          *duration = MCI_GetLastRampFinalDuration(pMCI) ;
        }
        break;
      break;
      case MC_REG_CURRENT_REF:
      {
        uint16_t *iqref = (uint16_t *) rawData;
        uint16_t *idref = (uint16_t *) &rawData[2];
        *rawSize = 4;
        *iqref = MCI_GetIqdref(pMCI).q;
        *idref = MCI_GetIqdref(pMCI).d;
      }
        break;
      case MC_REG_ASYNC_UARTA:
      case MC_REG_ASYNC_UARTB:
      case MC_REG_ASYNC_STLNK:

      default:
        retVal = MCP_ERROR_UNKNOWN_REG;
      }

      /* Size of the answer is size of the data + 2 bytes containing data size*/
      *size = (*rawSize)+2;
    }
    break;
  default:
    retVal = MCP_ERROR_BAD_DATA_TYPE;
    break;
  }
  return retVal;
}

uint8_t RI_MovString (const char * srcString, char * destString, uint16_t *size, uint16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;
  *size= 1 ; /* /0 is the min String size */
  while ((*srcString != 0) && (*size < maxSize) )
  {
    *destString = *srcString ;
    srcString = srcString+1;
    destString = destString+1;
    *size=*size+1;
  }
  if (*srcString != 0)
  { /* Last string char must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *destString = 0;
  }
return retVal;
}

uint8_t RI_GetIDSize (uint16_t dataID)
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t result;
  switch (typeID)
  {
    case TYPE_DATA_8BIT:
      result = 1;
      break;
    case TYPE_DATA_16BIT:
      result = 2;
      break;
    case TYPE_DATA_32BIT:
      result = 4;
      break;
    default:
      result=0;
      break;
  }
  return result;
}
__weak uint8_t RI_GetPtrReg (uint16_t dataID, void ** dataPtr)
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t motorID = (dataID & MOTOR_MASK)-1;
  uint16_t regID = dataID & REG_MASK;
  uint8_t retVal = MCP_CMD_OK;

  MCI_Handle_t * pMCI = &Mci[motorID];

  switch (typeID)
  {
  case TYPE_DATA_16BIT:
    {
      switch (regID)
      {
      case MC_REG_I_A:
        *dataPtr = &(pMCI->pFOCVars->Iab.a);
       break;
      case MC_REG_I_B:
        *dataPtr = &(pMCI->pFOCVars->Iab.b);
       break;
      case MC_REG_I_ALPHA:
        *dataPtr = &(pMCI->pFOCVars->Ialphabeta.alpha);
       break;
      case MC_REG_I_BETA:
        *dataPtr = &(pMCI->pFOCVars->Ialphabeta.beta);
       break;
      case MC_REG_I_Q:
        *dataPtr = &(pMCI->pFOCVars->Iqd.q);
       break;
      case MC_REG_I_D:
        *dataPtr = &(pMCI->pFOCVars->Iqd.d);
       break;
      case MC_REG_I_Q_REF:
        *dataPtr = &(pMCI->pFOCVars->Iqdref.q);
       break;
      case MC_REG_I_D_REF:
        *dataPtr = &(pMCI->pFOCVars->Iqdref.d);
       break;
      case MC_REG_V_Q:
        *dataPtr = &(pMCI->pFOCVars->Vqd.q);
       break;
      case MC_REG_V_D:
        *dataPtr = &(pMCI->pFOCVars->Vqd.d);
       break;
      case MC_REG_V_ALPHA:
        *dataPtr = &(pMCI->pFOCVars->Valphabeta.alpha);
       break;
      case MC_REG_V_BETA:
        *dataPtr = &(pMCI->pFOCVars->Valphabeta.beta);
       break;
/*
      case MC_REG_PRIMSENS_ANGLE:
        *dataPtr = &(pMCI->pFOCVars->hElAngle);
        break;
      case MC_REG_PRIMSENS_SPEED:
        break;
      case MC_REG_AUXSENS_ANGLE:
        break;
      case MC_REG_AUXSENS_SPEED:
        break;
*/
      default:
        break;
      }
    }
      break;
    default:

      break;
  }

  return retVal;
}
