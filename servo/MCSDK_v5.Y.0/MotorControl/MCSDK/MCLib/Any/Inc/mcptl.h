#ifndef MC_TRANSPORT_LAYER
#define MC_TRANSPORT_LAYER

#include "stdint.h"
#include "stdbool.h"
#undef MCP_DEBUG_METRICS
#define MCTL_SYNC  0xA
#define MCTL_ASYNC 0x9

#define MCTL_SYNC_NOT_EXPECTED 1

typedef struct MCTL_Handle MCTL_Handle_t;

typedef bool (* MCTL_GetBuf ) (MCTL_Handle_t *pHandle, void **buffer, uint8_t syncAsync);
typedef uint8_t (* MCTL_SendPacket ) (MCTL_Handle_t *pHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync);
typedef uint8_t* (* MCTL_RXpacketProcess ) (MCTL_Handle_t *pHandle, uint16_t *packetLength);

typedef enum {
  free = 0,
  writeLock = 1,
  pending = 2,
  readLock =3,
}buff_access_t;

typedef struct {
  uint8_t *buffer;
  uint16_t length;
  buff_access_t state;
#ifdef MCP_DEBUG_METRICS
   /* Debug metrics */
  uint16_t SentNumber;
  uint16_t PendingNumber;
  uint16_t RequestedNumber;
  /* End of Debug metrics */
#endif
}MCTL_Buff_t;

struct MCTL_Handle
{
  MCTL_GetBuf fGetBuffer; 
  MCTL_SendPacket fSendPacket;
  MCTL_RXpacketProcess fRXPacketProcess;
  uint16_t txSyncMaxPayload;
  uint16_t txAsyncMaxPayload;
  bool MCP_PacketAvailable; /* Packet available for Motor control protocol*/
} ;

bool MCTL_decodeCRCData (MCTL_Handle_t *pHandle);

#endif