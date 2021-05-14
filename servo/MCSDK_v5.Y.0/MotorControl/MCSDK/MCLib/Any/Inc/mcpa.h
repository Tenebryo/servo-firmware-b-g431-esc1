#ifndef ASYNC_CTRL_H
#define ASYNC_CTRL_H

#include "stdint.h"
#include "mcptl.h"
extern uint32_t GLOBAL_TIMESTAMP;



typedef struct 
{
  MCTL_Handle_t * pTransportLayer;
  void * pHFTableID[6];
  void * pMFTableID[6];
  void * pHFTableIDBuff[6];
  void * pMFTableIDBuff[6];
  uint8_t *currentBuffer;
  uint16_t bufferIndex;
  uint16_t bufferIndexMax;
  uint16_t bufferIndexMaxBuff;
  uint16_t bufferTxTrigger;
  uint16_t bufferTxTriggerBuff;
  #ifdef MCP_DEBUG_METRICS
  uint16_t bufferMissed;
  #endif
  uint8_t MFIDSize[6];
  uint8_t MFIDSizeBuff[6];  
  uint8_t HFIndex;
  uint8_t MFIndex;
  uint8_t HFRate;
  uint8_t HFRateBuff;  
  uint8_t HFNum; 
  uint8_t HFNumBuff;   
  uint8_t MFRate;
  uint8_t MFRateBuff;
  uint8_t MFNum;
  uint8_t MFNumBuff;  
  uint8_t Mark;
  uint8_t MarkBuff;  
} MCPA_Handle_t; /* MCP Async handle type*/


void MCPA_dataLog (MCPA_Handle_t *pHandle);

uint8_t MCPA_cfgLog (MCPA_Handle_t *pHandle, uint8_t * cfgdata);

#endif