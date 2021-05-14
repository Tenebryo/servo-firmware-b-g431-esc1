
/**
  ******************************************************************************
  * @file
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include <stdint.h>
#include "aspep.h"

#define MIN(a,b) ( (a < b) ? a : b )

uint8_t ASPEP_TXframeProcess (ASPEP_Handle_t *pHandle, uint8_t packetType, void *txBuffer, uint16_t bufferLength);
void ASPEP_sendBeacon (ASPEP_Handle_t *pHandle, ASPEP_Capabilities_def *capabilities);
void ASPEP_sendPing (ASPEP_Handle_t *pHandle, uint8_t state, uint16_t PacketNumber);

/**
 * @brief CRC-4 lookup table with 16 entries
 *
 *  Used to compute and check the CRC on the header with either the BYTE or NIBBLE granularity
 * In case the BYTE granularity is used, the table is used only with the 7th nibble of the header
 * (bits 24 to 27) when computing the CRC.
 */
uint8_t const CRC4_Lookup4[] = {
  /*   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   A,     B,    C,    D,    E,    F */
    0x00, 0x07, 0x0e, 0x09, 0x0b, 0x0c, 0x05, 0x02, 0x01, 0x06, 0x0f, 0x08, 0x0a, 0x0d, 0x04, 0x03
};

#ifndef CRC4_PER_NIBBLE
/**
 * @brief CRC-4 lookup table with 256 entries
 *
 *  Used to compute and check the CRC on the header with the BYTE granularity.
 *
 * When computing the CRC, the 7th nibble must to be processed with the CRC_Lookup4
 * table. Indeed, the header contains 7 nibbles (28 bits) of data on which the CRC is
 * to be computed. Hence, the CRC4_Lookup8 table cannot be used for the last nibble.
 *
 * Using only this lookup table requires that the amount of input data is a multiple of
 * 8 bits.
 */
uint8_t const CRC4_Lookup8[] = {
  /*          0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   A,     B,    C,    D,    E,    F */
  /* 0 */  0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e, 0x07, 0x05, 0x03, 0x01, 0x0f, 0x0d, 0x0b, 0x09,
  /* 1 */  0x07, 0x05, 0x03, 0x01, 0x0f, 0x0d, 0x0b, 0x09, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e,
  /* 2 */  0x0e, 0x0c, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x00, 0x09, 0x0b, 0x0d, 0x0f, 0x01, 0x03, 0x05, 0x07,
  /* 3 */  0x09, 0x0b, 0x0d, 0x0f, 0x01, 0x03, 0x05, 0x07, 0x0e, 0x0c, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x00,
  /* 4 */  0x0b, 0x09, 0x0f, 0x0d, 0x03, 0x01, 0x07, 0x05, 0x0c, 0x0e, 0x08, 0x0a, 0x04, 0x06, 0x00, 0x02,
  /* 5 */  0x0c, 0x0e, 0x08, 0x0a, 0x04, 0x06, 0x00, 0x02, 0x0b, 0x09, 0x0f, 0x0d, 0x03, 0x01, 0x07, 0x05,
  /* 6 */  0x05, 0x07, 0x01, 0x03, 0x0d, 0x0f, 0x09, 0x0b, 0x02, 0x00, 0x06, 0x04, 0x0a, 0x08, 0x0e, 0x0c,
  /* 7 */  0x02, 0x00, 0x06, 0x04, 0x0a, 0x08, 0x0e, 0x0c, 0x05, 0x07, 0x01, 0x03, 0x0d, 0x0f, 0x09, 0x0b,
  /* 8 */  0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f, 0x06, 0x04, 0x02, 0x00, 0x0e, 0x0c, 0x0a, 0x08,
  /* 9 */  0x06, 0x04, 0x02, 0x00, 0x0e, 0x0c, 0x0a, 0x08, 0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f,
  /* A */  0x0f, 0x0d, 0x0b, 0x09, 0x07, 0x05, 0x03, 0x01, 0x08, 0x0a, 0x0c, 0x0e, 0x00, 0x02, 0x04, 0x06,
  /* B */  0x08, 0x0a, 0x0c, 0x0e, 0x00, 0x02, 0x04, 0x06, 0x0f, 0x0d, 0x0b, 0x09, 0x07, 0x05, 0x03, 0x01,
  /* C */  0x0a, 0x08, 0x0e, 0x0c, 0x02, 0x00, 0x06, 0x04, 0x0d, 0x0f, 0x09, 0x0b, 0x05, 0x07, 0x01, 0x03,
  /* D */  0x0d, 0x0f, 0x09, 0x0b, 0x05, 0x07, 0x01, 0x03, 0x0a, 0x08, 0x0e, 0x0c, 0x02, 0x00, 0x06, 0x04,
  /* E */  0x04, 0x06, 0x00, 0x02, 0x0c, 0x0e, 0x08, 0x0a, 0x03, 0x01, 0x07, 0x05, 0x0b, 0x09, 0x0f, 0x0d,
  /* F */  0x03, 0x01, 0x07, 0x05, 0x0b, 0x09, 0x0f, 0x0d, 0x04, 0x06, 0x00, 0x02, 0x0c, 0x0e, 0x08, 0x0a
};
#endif /* CRC4_PER_NIBBLE */

/**
 * @brief Computes a 4-bit CRC on the 28 LSBs of @p header and returns it in the 4 MSB of the header
 *
 *  The generator polynomial used for the CRC is x^4+x+1 (ref. CCITT-G704).
 *
 *  The 28 input bits are split into 7 nibbles that are processed from the least significant to the
 * most significant one as follows:
 *
 *  - the least significant (4-bit) nibble is processed first as if it were the most significant part
 *    of the divident;
 *  - the order of bits in each nibble is unchanged for processing which would leads to the following
 *    bit processing sequence: 3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 15, 14, 13, 12, 19, 18, 17, 16,
 *    23, 22, 21, 20, 27, 26, 25, 24.
 *
 *  Two lookup tables based implementations are proposed: one solely relying on a 16 entries lookup
 * table and another one that also uses a 256 entries lookup table. This last one is the default. It
 * requires 256 more bytes than the first but is also roughly two times faster.
 *
 *  The 16 entries lookup table based implementation is enabled when the CRC4_PER_NIBBLE preprocessor
 * flag is defined.
 *
 *  Note that the default, 256 entries lookup table based implementation also uses the 16 entries lookup
 * table because the amount of input data is not a multiple of 8 bits.
 *
 * The resulting CRC is written in bits 28 to 31 of @p header and the whole header is returned.
 */
void ASPEP_ComputeHeaderCRC( uint32_t* headerPtr )
{
    uint8_t crc = 0;
    uint32_t header = *headerPtr;

#ifndef CRC4_PER_NIBBLE
    header &= 0x0fffffff;

    crc = CRC4_Lookup8[ crc ^ (uint8_t)( header        & 0xff) ];
    crc = CRC4_Lookup8[ crc ^ (uint8_t)((header >> 8 ) & 0xff) ];
    crc = CRC4_Lookup8[ crc ^ (uint8_t)((header >> 16) & 0xff) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 24) & 0x0f) ];
#else /* CRC4_PER_NIBBLE */
    crc = CRC4_Lookup4[ crc ^ (uint8_t)( header        & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >>  4) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >>  8) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 12) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 16) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 20) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 24) & 0xf) ];
#endif /* CRC4_PER_NIBBLE */

    *headerPtr |= (uint32_t)crc << 28;

    return;
}

/**
 * @brief Checks if @p header contains a valid 4-bit CRC and return true if this is the case and false otherwise
 *
 *  The CRC is computed over the 32 bits of the @p header. If the result is 0, the CRC is considered valid
 * and true is returned. If the result is not 0, false is returned.
 *
 * @sa ASPEP_ComputeHeaderCRC
 */
bool ASPEP_CheckHeaderCRC( uint32_t header )
{
    uint8_t crc = 0;

#ifndef CRC4_PER_NIBBLE
    crc = CRC4_Lookup8[ crc ^ (uint8_t)( header        & 0xff) ];
    crc = CRC4_Lookup8[ crc ^ (uint8_t)((header >> 8 ) & 0xff) ];
    crc = CRC4_Lookup8[ crc ^ (uint8_t)((header >> 16) & 0xff) ];
    crc = CRC4_Lookup8[ crc ^ (uint8_t)((header >> 24) & 0xff) ];
#else /* CRC4_PER_NIBBLE */
    crc = CRC4_Lookup4[ crc ^ (uint8_t)( header        & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >>  4) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >>  8) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 12) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 16) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 20) & 0xf) ];
    crc = CRC4_Lookup4[ crc ^ (uint8_t)((header >> 24) & 0xf) ];
    crc = crc ^ (uint8_t)((header >> 28) & 0xf);
#endif /* CRC4_PER_NIBBLE */

    return crc == 0;
}

void ASPEP_start(ASPEP_Handle_t *pHandle)
{
  pHandle->fASPEP_HWInit ( pHandle->HWIp );
  pHandle->ASPEP_State = ASPEP_IDLE;
  pHandle->ASPEP_TL_State = WAITING_PACKET;
  pHandle -> syncPacketCount = 0; /* Sync packet counter is reset only at startup*/

  /* Configure UART to receive first packet*/
  pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxHeader , ASPEP_HEADER_SIZE );
}

void ASPEP_sendBeacon (ASPEP_Handle_t *pHandle, ASPEP_Capabilities_def *capabilities)
{
  uint32_t * packet = (uint32_t*) pHandle->ctrlBuffer.buffer;
  *packet =( BEACON |
           (capabilities->version <<4) |
           (capabilities->DATA_CRC << 7) |
           (((uint32_t)capabilities->RX_maxSize) << 8) |
           (((uint32_t)capabilities->TXS_maxSize) << 14) |
           (((uint32_t)capabilities->TXA_maxSize) << 21));

  ASPEP_TXframeProcess (pHandle, ASPEP_CTRL, &pHandle->ctrlBuffer, ASPEP_CTRL_SIZE);
}

void ASPEP_sendNack (ASPEP_Handle_t *pHandle, uint8_t error)
{
  uint32_t * packet = (uint32_t*) pHandle->ctrlBuffer.buffer;
  *packet = NACK | (((uint32_t) error) << 8) |(((uint32_t) error) << 16);
  ASPEP_TXframeProcess (pHandle, ASPEP_CTRL, &pHandle->ctrlBuffer, ASPEP_CTRL_SIZE);
}

void ASPEP_sendPing (ASPEP_Handle_t *pHandle, uint8_t cBit, uint16_t packetNumber)
{
  uint32_t * packet = (uint32_t*) pHandle->ctrlBuffer.buffer;
  uint8_t Nbit = pHandle->syncPacketCount & 0x1; /* Keep only LSB */
  uint8_t ipID = pHandle->liid &0xF;
  *packet = PING|(cBit <<4)|(cBit <<5)|(Nbit<<6)|(Nbit<<7)|(ipID<<8)|(((uint32_t) packetNumber) <<12);
  ASPEP_TXframeProcess (pHandle, ASPEP_CTRL, &pHandle->ctrlBuffer, ASPEP_CTRL_SIZE);
}

bool ASPEP_getBuffer (MCTL_Handle_t *pSupHandle, void **buffer,  uint8_t syncAsync)
{
  ASPEP_Handle_t *pHandle = (ASPEP_Handle_t *) pSupHandle;
  bool result = true;
  if (syncAsync == MCTL_SYNC)
  {
    if (pHandle->syncBuffer.state <= writeLock ) /* Possible values are free or writeLock*/
    {
      *buffer = &pHandle->syncBuffer.buffer[ASPEP_HEADER_SIZE];
      pHandle->syncBuffer.state = writeLock;
    }
    else
    {
      result = false;
    }
  }
  else /* Asynchronous buffer request */
  {
    if ((pHandle->asyncBufferA.state > writeLock) && (pHandle->asyncBufferB.state > writeLock ))
    {
      result = false;
    }
    else
    {
      if (pHandle->asyncBufferA.state <= writeLock)
      {
        pHandle->asyncBufferA.state = writeLock;
        pHandle->lastRequestedAsyncBuff = &pHandle->asyncBufferA;
        *buffer = &pHandle->asyncBufferA.buffer[ASPEP_HEADER_SIZE];
		#ifdef MCP_DEBUG_METRICS
        pHandle->asyncBufferA.RequestedNumber++;
		#endif
      }
      else if (pHandle->asyncBufferB.state <= writeLock)
      {
        pHandle->asyncBufferB.state = writeLock;
        pHandle->lastRequestedAsyncBuff = &pHandle->asyncBufferB;
        *buffer = &pHandle->asyncBufferB.buffer[ASPEP_HEADER_SIZE];
		#ifdef MCP_DEBUG_METRICS
        pHandle->asyncBufferB.RequestedNumber++;
		#endif
      }
    }
  }
return result;
}

bool ASPEP_CheckBeacon (ASPEP_Handle_t * pHandle, ASPEP_Capabilities_def *MasterCapabilities)
{
  uint32_t packetHeader = *((uint32_t *)pHandle->rxHeader);
  bool result = true;
  MasterCapabilities->version =  (packetHeader &0x70)>> 4; /*Bits 4 to 6*/
  MasterCapabilities->DATA_CRC = pHandle->rxHeader[0] >> 7 ;      /*Bit 7 */
  MasterCapabilities->RX_maxSize =  pHandle->rxHeader[1] &0x3F; /*Bits 8 to  13*/
  MasterCapabilities->TXS_maxSize = (packetHeader&0x1FC000)  >> 14; /*Bits 14 to 20 */
  MasterCapabilities->TXA_maxSize = (packetHeader&0xFE00000) >> 21; /*Bits 21 to 27  */
  if ((MasterCapabilities->DATA_CRC != pHandle->Capabilities.DATA_CRC) ||
      (MasterCapabilities->RX_maxSize > pHandle->Capabilities.RX_maxSize) || /* Data packet the master can send is bigger than slave can receive */
      (pHandle->Capabilities.TXS_maxSize != MasterCapabilities->TXS_maxSize ) || /*Sync packet size alignement is required in order for the master to be able to store it, and to not request a response bigger than slave capability */
      (pHandle->Capabilities.TXA_maxSize != MasterCapabilities->TXA_maxSize ) || /*Async packet the slave can send is bigger than the master can receive (but master must not request a bigger packet than slave can transmit */
      (MasterCapabilities->version != pHandle->Capabilities.version) ) /* For the time being, master version has to match slave version */
  {
    result = false;
  }
  return result;
}

/*

 void *txBuffer, can be 8, 16 or 32 bits, but must be 32 bits aligned.
 uint16_t txDataLength Size of useful data in byte

*/

uint8_t ASPEP_sendPacket (MCTL_Handle_t *pSupHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync)
{
  uint8_t *packet;
  uint32_t *header;
  uint32_t tmpHeader;
  uint8_t result = ASPEP_OK;
  ASPEP_Handle_t *pHandle = (ASPEP_Handle_t *) pSupHandle;

  if (pHandle-> ASPEP_State == ASPEP_CONNECTED)
  {

    /*We must add packet header on  */
    /* | [0101|0011] | Length 13b | Reserved |CRCH 4b| */
    packet = (uint8_t *) txBuffer;
    packet = packet - ASPEP_HEADER_SIZE; /* Header ues 4*8 bits on top of txBuffer*/
    header = (uint32_t *) packet;
    tmpHeader =  (txDataLength << 4) | syncAsync;
    *header = tmpHeader;
    if (pHandle->Capabilities.DATA_CRC == 1)
    {
      /* TODO : Compute real CRC*/
      *(packet+ASPEP_HEADER_SIZE+txDataLength) = (uint8_t) 0xCA; /* Dummy CRC */
      *(packet+ASPEP_HEADER_SIZE+txDataLength+1) = (uint8_t) 0xFE; /* Dummy CRC */
      txDataLength+=ASPEP_DATACRC_SIZE;
    }
    if (syncAsync == MCTL_SYNC )
    {
      if ( pSupHandle->MCP_PacketAvailable)
      {
        pSupHandle-> MCP_PacketAvailable = false; /* CMD from master is processed*/
      }
      else
      {
        result = MCTL_SYNC_NOT_EXPECTED;
      }
    }
    if (result == ASPEP_OK) /* Send packet only if no error detected so far*/
    {
      result = ASPEP_TXframeProcess (pHandle, syncAsync, packet, txDataLength+ASPEP_HEADER_SIZE);
    }
  }
  else
  {
    result = ASPEP_NOT_CONNECTED;
  }
  return result;
}

/*
 This function contains a critical section.
 It can be accessed concurently under High frequency task (by MCPA_datalog)
 and under Medium frequency task (MC_Scheduler -> ASPEP_RxFrameProcess )

dataType can be Synchronous (answer to Master packet), Asynchronous or a CTL packet

*/

uint8_t ASPEP_TXframeProcess (ASPEP_Handle_t *pHandle, uint8_t dataType, void *txBuffer, uint16_t bufferLength)
{
  uint8_t result = ASPEP_OK;
  ASPEP_ComputeHeaderCRC  ((uint32_t*)txBuffer); /* Insert CRC header in the packet to send */
  __disable_irq(); /*TODO: Disable High frequency task is enough */
  if (pHandle->lockBuffer == NULL ) /* Communication Ip free to send data*/
  {
    if (dataType == MCTL_ASYNC )
    {
      /* In case of ASYNC, two flipflop buffers are used, the txBuffer points always to lastRequestedAsyncBuff->buffer */
      pHandle->lastRequestedAsyncBuff->state = readLock;
      pHandle->lockBuffer = (void *) pHandle->lastRequestedAsyncBuff;
	  #ifdef MCP_DEBUG_METRICS
      pHandle->lastRequestedAsyncBuff->SentNumber++;
	  #endif
    }
    else if (dataType == MCTL_SYNC )
    {
      pHandle->syncBuffer.state = readLock;
      pHandle->lockBuffer = (void *) &pHandle->syncBuffer;
    }
    else
    {
      pHandle->ctrlBuffer.state = readLock;
      pHandle->lockBuffer = (void *) &pHandle->ctrlBuffer;
    }
    /* Enable HF task It */
    __enable_irq(); /*TODO: Enable High frequency task is enough */
    pHandle->fASPEP_send  (pHandle ->HWIp, txBuffer, bufferLength);
  }
  else /* HW resource busy, saving packet to sent it once resource will be freed*/
  {
    __enable_irq(); /*TODO: Enable High frequency task is enough */
    /* Lock buffer can be freed here */
    if ( dataType == MCTL_ASYNC )
    {
      /* Check that the buffer received is the one expected - probably useless */
      if ( txBuffer != (uint8_t *) pHandle->lastRequestedAsyncBuff->buffer)
      {
        result = ASPEP_BUFFER_ERROR;
      }
      if (pHandle->asyncNextBuffer == NULL)
      { /* Required to keep the right sending order */
        pHandle->asyncNextBuffer = pHandle->lastRequestedAsyncBuff;
      }
	    else
	    { /* nothing to do */
	    }
      pHandle->lastRequestedAsyncBuff->state = pending;
      pHandle->lastRequestedAsyncBuff->length = bufferLength;
	  #ifdef MCP_DEBUG_METRICS
      pHandle->lastRequestedAsyncBuff->PendingNumber++;
	  #endif
    }
    else if (dataType == MCTL_SYNC )
    {
      if ( pHandle -> syncBuffer.state != writeLock ) {
        result = ASPEP_BUFFER_ERROR;
      }
      else {
        pHandle -> syncBuffer.state = pending;
        pHandle -> syncBuffer.length = bufferLength;
      }
    }
    else  if ( dataType == ASPEP_CTRL )
    {
     if ( pHandle -> ctrlBuffer.state != free ) {
        result = ASPEP_BUFFER_ERROR;
      }
      else {
        pHandle -> ctrlBuffer.state = pending;
      }
    }
  }
return result;
}

/* ASPEP_HWDataTransmittedIT is called as soon as previous packet transfer is completed */
/* pHandle->lockBuffer is set before packet transmission and is never read here after */
/* therefore, there is no need to protect this ISR against another higher priority ISR (HF Task)*/

void ASPEP_HWDataTransmittedIT (ASPEP_Handle_t *pHandle )
{
  /* First free previous readLock buffer */
  if (pHandle->ctrlBuffer.state == readLock)
  {
    pHandle -> ctrlBuffer.state = free;
  }
  else /* if previous buffer was not ASPEP_CTRL, then the buffer locked is a MCTL_Buff_t */
  {
    MCTL_Buff_t * tempBuff = (MCTL_Buff_t *) pHandle -> lockBuffer;
    tempBuff->state = free;
  }
  if ( pHandle -> syncBuffer.state == pending )
  {
    pHandle->lockBuffer = (void *) &pHandle->syncBuffer;
    pHandle->fASPEP_send (pHandle->HWIp, pHandle->syncBuffer.buffer, pHandle->syncBuffer.length);
    pHandle ->syncBuffer.state = readLock;
  }
  /* Second prepare transfer of pending buffer */
  else if ( pHandle -> ctrlBuffer.state == pending )
  {
    pHandle->lockBuffer = (void *)(&pHandle ->ctrlBuffer);
    pHandle->fASPEP_send (pHandle ->HWIp, pHandle->ctrlBuffer.buffer, ASPEP_CTRL_SIZE);
    pHandle -> ctrlBuffer.state = readLock;
  }
  else
  {
    __disable_irq();
    if ( pHandle->asyncNextBuffer != NULL )
    {
      pHandle->lockBuffer = (void *) pHandle->asyncNextBuffer;
      pHandle->asyncNextBuffer->state = readLock;
	  #ifdef MCP_DEBUG_METRICS
      pHandle->asyncNextBuffer->SentNumber++;
	  #endif
      pHandle->fASPEP_send (pHandle ->HWIp, pHandle->asyncNextBuffer->buffer, pHandle->asyncNextBuffer->length);
      /* If one Async buffer is still pending, assign it to the asyncNextBuffer pointer*/
      if ( (pHandle->asyncBufferA.state == pending ) || (pHandle->asyncBufferB.state == pending))
      {
        uint32_t temp = (uint32_t) &pHandle->asyncBufferA + (uint32_t) &pHandle->asyncBufferB - (uint32_t)  pHandle->asyncNextBuffer;
        pHandle->asyncNextBuffer = (MCTL_Buff_t *) temp;
      }
      else {
        pHandle->asyncNextBuffer = NULL;
      }
    }
    else /* No TX packet are pending, HW resource is free*/
    {
      pHandle->lockBuffer = NULL;
    }
    __enable_irq();
  }
}

uint8_t* ASPEP_RXframeProcess (MCTL_Handle_t *pSupHandle, uint16_t *packetLength)
{
  uint8_t* result = NULL;
  ASPEP_Handle_t * pHandle = (ASPEP_Handle_t *) pSupHandle;
  bool validCRCData = true;
  ASPEP_Capabilities_def MasterCapabilities;
  uint16_t packetNumber;
  uint32_t packetHeader = *((uint32_t *)pHandle->rxHeader);

  *packetLength = 0;
  if (pHandle->NewPacketAvailable)
  {
    pHandle -> NewPacketAvailable = false; /* Consumes new packet*/
    switch (pHandle->ASPEP_State)
    {
    case ASPEP_IDLE:
      if (pHandle->rxPacketType == beacon )
      {
        if (ASPEP_CheckBeacon(pHandle,&MasterCapabilities) == false)
        {
          // pHandle->Capabilities.RX_maxSize =  MIN(pHandle->Capabilities.RX_maxSize,  MasterCapabilities.RX_maxSize);
          pHandle->Capabilities.DATA_CRC = MIN(pHandle->Capabilities.DATA_CRC ,MasterCapabilities.DATA_CRC);
          pHandle->Capabilities.TXS_maxSize = MIN(pHandle->Capabilities.TXS_maxSize, MasterCapabilities.TXS_maxSize);
          pHandle->Capabilities.TXA_maxSize = MIN(pHandle->Capabilities.TXA_maxSize, MasterCapabilities.TXA_maxSize);
        }
        else
        { /* Master capabilities match Salve capabilities.*/
          pSupHandle->txSyncMaxPayload = (pHandle->Capabilities.TXS_maxSize+1)*32;
          pSupHandle->txAsyncMaxPayload = (pHandle->Capabilities.TXA_maxSize)*64;
          pHandle->maxRXPayload = (pHandle->Capabilities.RX_maxSize+1)*32;
          pHandle->ASPEP_State = ASPEP_CONFIGURED;
        }
        /* Beacon Packet must be answered*/
          ASPEP_sendBeacon (pHandle, &pHandle->Capabilities);
      }
      else if (pHandle->rxPacketType == ping)
      { /* In Listening for master slave, */
        packetNumber = (packetHeader&0x0FFFF000 ) >> 12;
        ASPEP_sendPing (pHandle,ASPEP_PING_RESET,packetNumber);
      }
      else
      {
      }
    break;
    case ASPEP_CONFIGURED:
      if (pHandle->rxPacketType == beacon )
      {
        if (ASPEP_CheckBeacon(pHandle,&MasterCapabilities) == false)
        {
          pHandle->ASPEP_State = ASPEP_IDLE;
        }
        else
        { /* Nothing to do */
        }
        ASPEP_sendBeacon (pHandle, &pHandle->Capabilities);
      }
      else if (pHandle->rxPacketType == ping)
      { /* In Listening for master slave, */
        packetNumber = (packetHeader&0x0FFFF000 ) >> 12;
        ASPEP_sendPing (pHandle,ASPEP_PING_CFG,packetNumber);
        pHandle->ASPEP_State = ASPEP_CONNECTED;
      }
    break;
    case ASPEP_CONNECTED:
      if (pHandle->rxPacketType == beacon )
      {
        if (ASPEP_CheckBeacon(pHandle,&MasterCapabilities) == false)
        {
          pHandle->ASPEP_State = ASPEP_IDLE;
        }
        else
        {
          pHandle->ASPEP_State = ASPEP_CONFIGURED;
        }
        ASPEP_sendBeacon (pHandle, &pHandle->Capabilities);
      }
      else if (pHandle->rxPacketType == ping )
      {
        packetNumber = pHandle->rxHeader[1];
        ASPEP_sendPing (pHandle,ASPEP_PING_CFG,packetNumber);
      }
      else if (pHandle->rxPacketType == data )
      {
        if (validCRCData)
        { pHandle -> syncPacketCount++; /* this counter is incremented at each valid data packet received from the master */
          pSupHandle ->MCP_PacketAvailable = true; /* Will be consumed in ASPEP_sendPacket */
          *packetLength = pHandle->rxLength;
          result = pHandle->rxBuffer;
        }
        else
        {
          ASPEP_sendNack (pHandle, ASPEP_BAD_CRC_DATA);
        }
      }
      else {
        /* This condition is not reachable because already filtred by NewPacketAvailable */
        /* ASPEP_sendNack (pHandle, ASPEP_BAD_PACKET_TYPE); */
      }

      break;
    }
    /* The valid received packet is now safely consumes, we are ready to receive a new packet*/
    pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxHeader , ASPEP_HEADER_SIZE );
  }
  else if (pHandle->badPacketFlag > ASPEP_OK )
  {
    ASPEP_sendNack (pHandle, pHandle->badPacketFlag);
    /* ASPEP_RXframeProcess can be called before reception of another packet*/
    pHandle->badPacketFlag = ASPEP_OK;
    /* As we received a packet with a bad header, we need to be sure that the HW IP is well Synchronised*/
    /* DMA will be configured to receive next packet as soon as HW IP RX line is free to receive new packet */
    /* It is important to note that we will detect only the NEXT free line transition, it means the next packet will be lost*/
    /* but the end of this lost packet will generate the IDLE interrupt */
    /* the IDLE interrupt will call ASPEP_HWDMAReset (in charge of the IP_aspep driver to call it at the appropriate time)*/
    pHandle->fASPEP_HWSync (pHandle->HWIp);
  }
  else
  {
    /* Nothing to do, no response is due to the master */
  }
  return result;
}

/* This function is called once DMA has transfered the configure number of byte*/
void ASPEP_HWDataReceivedIT (ASPEP_Handle_t *pHandle)
{
/* Upon reception of a Newpacket the DMA will be re-configured only once the answer has been sent.*/
/* This is mandatory to avoid a race condition in case of a new packet is received while executing ASPEP_RXframeProcess*/
/* If the packet received contains an error in the header, the HW IP will be re-synchronised first, and DMA will be configured after.*/
  switch (pHandle->ASPEP_TL_State) {
  case WAITING_PACKET:
    if (ASPEP_CheckHeaderCRC (*(uint32_t*)pHandle->rxHeader) == true )
    {
       pHandle->rxPacketType = (ASPEP_packetType_def) (pHandle->rxHeader[0] & ID_MASK);
       switch (pHandle->rxPacketType) {
       case data:
         pHandle->rxLength = (*((uint32_t *)pHandle->rxHeader) & 0x0001FFF0) >> 4;
         if (pHandle->rxLength == 0) /* data packet with length 0 is a valid packet*/
         {
           pHandle->NewPacketAvailable = true;
           /*The receiver is not reconfigure right now on purpose to avoid race condition when the packet will be processed in ASPEP_RXframeProcess */
         }
         else if (pHandle->rxLength <= pHandle->maxRXPayload)
         {
           pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxBuffer , pHandle->rxLength+ASPEP_DATACRC_SIZE*pHandle->Capabilities.DATA_CRC); /* need to read + 2 bytes CRC*/
           pHandle->ASPEP_TL_State = WAITING_PAYLOAD;
         }
         else
         {
           pHandle->badPacketFlag = ASPEP_BAD_PACKET_SIZE;
         }
         break;
       case beacon:
       case ping:
         pHandle->NewPacketAvailable = true;
         /*The receiver is not reconfigure right now on purpose to avoid race condition when the packet will be processed in ASPEP_RXframeProcess */
        break;
       default:
         pHandle->badPacketFlag = ASPEP_BAD_PACKET_TYPE;
         break;
       }
    }
    else
    {
      pHandle->badPacketFlag = ASPEP_BAD_CRC_HEADER;
    }
    break;
  case WAITING_PAYLOAD:
    pHandle->ASPEP_TL_State = WAITING_PACKET;
    // Payload received,
    pHandle->NewPacketAvailable = true;
    /*The receiver is not reconfigure right now on purpose to avoid race condition when the packet will be processed in ASPEP_RXframeProcess */
    break;
  }
}

/* Called after debugger has stopped the MCU*/
void ASPEP_HWDMAReset (ASPEP_Handle_t *pHandle)
{ /* We must reset the RX state machine to be sure to not be in Waiting packet state */
  /* Otherwise the arrival of a new packet will trigger a NewPacketAvailable despite */
  /* the fact that bytes have been lost because of overrun (debugger paused for instance) */
  pHandle->ASPEP_TL_State = WAITING_PACKET;
  pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxHeader , ASPEP_HEADER_SIZE );
}
