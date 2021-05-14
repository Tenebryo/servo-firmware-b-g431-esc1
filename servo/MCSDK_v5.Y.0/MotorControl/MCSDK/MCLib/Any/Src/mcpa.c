#include "string.h"
#include "mcp.h"
#include "register_interface.h"
#include "mcpa.h"

uint32_t GLOBAL_TIMESTAMP = 0;

void MCPA_dataLog (MCPA_Handle_t *pHandle)
{
  uint8_t i;
  uint16_t *logValue16;
  uint32_t *logValue;
  
  if (pHandle->HFIndex == pHandle->HFRateBuff) /*  */
  {
    pHandle->HFIndex = 0;
    if (pHandle->bufferIndex == 0)
    {
      /* New buffer allocation */
      if ( pHandle->pTransportLayer->fGetBuffer (pHandle->pTransportLayer, (void **) &pHandle->currentBuffer, MCTL_ASYNC))
      {
        logValue = (uint32_t *) pHandle->currentBuffer;
        *logValue = GLOBAL_TIMESTAMP; /* 32 first bits is used to store Timestamp */
        pHandle->bufferIndex = 4; 
        pHandle->MFIndex = 0; /* Restart the motif from scratch at each buffer*/
        /* Check if configuration has changed for this new buffer */
        if (pHandle->Mark != pHandle->MarkBuff)
        {
          pHandle->MarkBuff = pHandle->Mark;
          pHandle->HFNumBuff = pHandle->HFNum;
          pHandle->MFNumBuff = pHandle->MFNum;
          pHandle->HFRateBuff = pHandle->HFRate;
          pHandle->MFRateBuff = pHandle->MFRate;
          pHandle->bufferTxTriggerBuff = pHandle->bufferTxTrigger;
          memcpy(pHandle->pHFTableIDBuff, pHandle->pHFTableID, pHandle->HFNum*4); /* We store pointer here, so 4 bytes */
          memcpy(pHandle->pMFTableIDBuff, pHandle->pMFTableID, pHandle->MFNum*4);
          memcpy(pHandle->MFIDSizeBuff, pHandle->MFIDSize, pHandle->MFNum ); /* 1 size byte per MF ID*/
        }
      }
      else
      {
        /* Nothing to do, try next HF Task to get an Async buffer*/
		#ifdef MCP_DEBUG_METRICS
          pHandle->bufferMissed++;
	    #endif
      }
    }  
    /* */
    if ((pHandle->bufferIndex > 0)  && (pHandle->bufferIndex <= pHandle->bufferTxTriggerBuff ))
    {
      logValue16 = (uint16_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
      for (i=0; i<pHandle->HFNumBuff; i++)
      {
        /* Dump HF data for now HF data are forced to 16 bits*/
        *logValue16 = *((uint16_t *) pHandle->pHFTableIDBuff[i]);
        logValue16 = logValue16+1;
        pHandle->bufferIndex = pHandle->bufferIndex+2;
      }
      /* MFRateBuff=254 means we dump MF data once per buffer */
      /* MFRateBuff=255 means we do not dump MF data */
      if (pHandle->MFRateBuff < 254) 
      {
        if (pHandle->MFIndex == pHandle->MFRateBuff)  
        {
          pHandle->MFIndex = 0;
          for (i=0; i<pHandle->MFNumBuff; i++)
          {
            /* Dump MF data*/ 
            /* we always cast to uint32_t whatever the data to copy.
               This strategy allow us to avoid a switch case which cost more
               than unaligned 32 bit access. 
               It is up to the master to optimize the structure alignement */          
            logValue = (uint32_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
            *logValue = *((uint32_t *) pHandle->pMFTableIDBuff[i]);
          /*  
            switch (pHandle->MFIDSizeBuff[i])
            {
              case 1:
                logValue8 = (uint8_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
                *logValue8 = *((uint8_t *) pHandle->pMFTableIDBuff[i]);
              break;              
              case 2:
                logValue16 = (uint16_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
                *logValue16 = *((uint16_t *) pHandle->pMFTableIDBuff[i]);
              break;              
              case 4:
                logValue32 = (uint32_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
                *logValue32 = *((uint32_t *) pHandle->pMFTableIDBuff[i]);
              break;
            }
            */
            pHandle->bufferIndex = pHandle->bufferIndex+pHandle->MFIDSizeBuff[i];
          }
        }
        else
        {
          pHandle->MFIndex ++;
        }
      }
    }
    if (pHandle->bufferIndex > pHandle->bufferTxTriggerBuff)
    {
      if (pHandle->MFRateBuff == 254) /* MFRateBuff = 254 means we dump MF data once per buffer */
      {
        for (i=0; i<pHandle->MFNumBuff; i++)
        {
         logValue = (uint32_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
         *logValue = *((uint32_t *) pHandle->pMFTableIDBuff[i]);
         pHandle->bufferIndex = pHandle->bufferIndex+pHandle->MFIDSizeBuff[i];
        }
      }
      /* Buffer is ready to be send*/
       logValue16 = (uint16_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
       *logValue16 = pHandle->MarkBuff; /* MarkBuff is actually 8 bits, but we add also 8 bits of the ASYNCID=0 after the MARK*/
       pHandle->pTransportLayer->fSendPacket (pHandle->pTransportLayer, pHandle->currentBuffer, pHandle->bufferIndex+2, MCTL_ASYNC);
       pHandle->bufferIndex = 0;
    }     
  }
  else 
  {
    /* nothing to log just waiting next call to MCPA_datalog*/
    pHandle->HFIndex++;
  }
}  
  
uint8_t MCPA_cfgLog (MCPA_Handle_t *pHandle, uint8_t * cfgdata)
{
  uint16_t newID, buffSize;
  uint8_t i;
  uint8_t logSize=0; /* Max size of a log per iteration (HF+MF)*/
  uint16_t *logValue16;
  uint8_t result = MCP_CMD_OK;
  
  buffSize = *((uint16_t *) cfgdata);
  if (buffSize > pHandle->pTransportLayer->txAsyncMaxPayload )
  {
    result = MCP_ERROR_NO_TXASYNC_SPACE;
  }
  else
  {
     pHandle->HFRate = *((uint8_t *) &cfgdata[2]);
     pHandle->HFNum  = *((uint8_t *) &cfgdata[3]);
     pHandle->MFRate = *((uint8_t *) &cfgdata[4]);
     pHandle->MFNum =  *((uint8_t *) &cfgdata[5]);
     cfgdata = &cfgdata[6]; /* Start of the HF IDs*/
     for (i =0; i <  pHandle->HFNum ; i++)
     {
       newID = *((uint16_t *) cfgdata);
       RI_GetPtrReg (newID, &pHandle->pHFTableID[i]);
       cfgdata = cfgdata+2; /* Point to the next UID */
       logSize = logSize+2;
     }
     for (i =0; i <  pHandle->MFNum ; i++)
     {
       newID = *((uint16_t *) cfgdata);
       RI_GetPtrReg (newID, &pHandle->pMFTableID[i]);
       pHandle->MFIDSize[i] = RI_GetIDSize(newID);
       logSize = logSize + pHandle->MFIDSize[i];
       cfgdata = cfgdata+2; /* Point to the next UID */
     }
     if (buffSize < (logSize+2+4) ) /*smallest packet must be able to contain logSize Markbyte AsyncID and TimeStamp*/
     {
       result = MCP_ERROR_NO_TXASYNC_SPACE;
     }
     else
     {
       pHandle->bufferTxTrigger = buffSize-logSize-2; /* 2 is required to add the last Mark byte and NUL ASYNCID */
       pHandle->Mark =   *((uint8_t *) cfgdata);
       if (pHandle->Mark == 0 && pHandle->MarkBuff != 0)
       {  /* Switch Off condition */
          /* if Mark is cleared, dataLog is disabled. We send buffer if not empty*/
          /* Not empty means Index>4 because 4 first bytes are used for TimeStamp*/
          if (pHandle->bufferIndex > 4) {  
            logValue16 = (uint16_t *) &pHandle->currentBuffer[pHandle->bufferIndex];
            *logValue16 = pHandle->MarkBuff; /* MarkBuff is actually 8 bits, but we add also 8 bits of the ASYNCID=0 after the MARK*/
            pHandle->pTransportLayer->fSendPacket (pHandle->pTransportLayer, pHandle->currentBuffer, pHandle->bufferIndex+2, MCTL_ASYNC);
          }  
          pHandle->bufferIndex = 0;
          pHandle->MarkBuff = 0;
       }
     }
  }
  return result;
}

  
