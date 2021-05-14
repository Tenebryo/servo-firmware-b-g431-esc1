#ifndef motor_control_protocol
#define motor_control_protocol

#include "stdint.h"
#include "mcptl.h"

/* Action suppoted by the Motor control protocol*/
#define MOTOR_MASK 0x7
#define CMD_MASK 0xFFF8

#define SET_DATA_ELEMENT 0x8
#define GET_DATA_ELEMENT 0x10
#define START_MOTOR      0x18
#define STOP_MOTOR       0x20
#define STOP_RAMP        0x28
#define START_STOP       0x30
#define FAULT_ACK        0x38 
#define ENCODER_ALIGN    0x40
#define IQDREF_CLEAR     0x48
#define PFC_ENABLE       0x50
#define PFC_DISABLE      0x58
#define PFC_FAULT_ACK    0x60
#define SW_RESET         0x78

/* MCP ERROR CODE */
#define MCP_CMD_OK      0x00
#define MCP_CMD_NOK     0x01
#define MCP_CMD_UNKNOWN 0x02
#define MCP_DATAID_UNKNOWN  0x03
#define MCP_ERROR_RO_REG 0x04
#define MCP_ERROR_UNKNOWN_REG 0x05
#define MCP_ERROR_STRING_FORMAT 0x06
#define MCP_ERROR_BAD_DATA_TYPE 0x07 
#define MCP_ERROR_NO_TXSYNC_SPACE 0x08
#define MCP_ERROR_NO_TXASYNC_SPACE 0x09
#define MCP_ERROR_BAD_RAW_FORMAT 0x0A

#define MCP_HEADER_SIZE 2

typedef struct 
{ 
  MCTL_Handle_t * pTransportLayer;
  uint8_t * rxBuffer;
  uint8_t * txBuffer;
  uint16_t rxLength;
  uint16_t txLength;

} MCP_Handle_t;

void MCP_ReceivedPacket(MCP_Handle_t * pHandle);
#endif