
#include "parameters_conversion.h"
#include "usart_aspep_driver.h"
#include "aspep.h"
#include "mcp.h"
#include "mcpa.h"
#include "mcp_config.h"

uint8_t MCPSyncTxBuff[MCP_TX_SYNCBUFFER_SIZE];
uint8_t MCPSyncRXBuff[MCP_RX_SYNCBUFFER_SIZE];

/* Asynchronous buffer dedicated to UART_A*/
uint8_t MCPAsyncBuffUARTA_A[MCP_TX_ASYNCBUFFER_SIZE_A];
uint8_t MCPAsyncBuffUARTA_B[MCP_TX_ASYNCBUFFER_SIZE_A];

UASPEP_Handle_t UASPEP_A =
{
 .USARTx = USARTA,
 .rxDMA = DMA_RX_A,
 .txDMA = DMA_TX_A,
 .rxChannel = DMACH_RX_A,
 .txChannel = DMACH_TX_A,
};

ASPEP_Handle_t aspepOverUartA =
{
  ._Super =
   {
    .fGetBuffer = &ASPEP_getBuffer,
    .fSendPacket = &ASPEP_sendPacket,
    .fRXPacketProcess = &ASPEP_RXframeProcess,
    },
  .HWIp = &UASPEP_A,
  .Capabilities = {
    .DATA_CRC = 0,
    .RX_maxSize =  (MCP_RX_SYNC_PAYLOAD_MAX>>5)-1,
    .TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX>>5)-1,
    .TXA_maxSize =  MCP_TX_ASYNC_PAYLOAD_MAX_A>>6,
    .version = 0x0,
  },
  .syncBuffer = {
   .buffer = MCPSyncTxBuff,
  },
  .asyncBufferA = {
    .buffer = MCPAsyncBuffUARTA_A,
  },
  .asyncBufferB = {
    .buffer = MCPAsyncBuffUARTA_B,
  },
  .rxBuffer = MCPSyncRXBuff,
  .fASPEP_HWInit = &UASPEP_INIT,
  .fASPEP_HWSync = &UASPEP_IDLE_ENABLE,
  .fASPEP_receive = &UASPEP_RECEIVE_BUFFER,
  .fASPEP_send = &UASPEP_SEND_PACKET,
  .liid = 0,
};

MCP_Handle_t MCP_Over_UartA =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartA,
};

MCPA_Handle_t MCPA_UART_A =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartA,
};

