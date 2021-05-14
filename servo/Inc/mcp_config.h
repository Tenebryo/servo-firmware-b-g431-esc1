
#ifndef MCP_CONFIG_H
#define MCP_CONFIG_H

#include "mcp.h"
#include "aspep.h"
#include "mcpa.h"

#define USARTA USART2
#define DMA_RX_A DMA2
#define DMA_TX_A DMA2
#define DMACH_RX_A LL_DMA_CHANNEL_2
#define DMACH_TX_A LL_DMA_CHANNEL_3
#define USARTA_IRQHandler USART2_IRQHandler
#define MCP_RX_IRQHandler_A DMA2_Channel2_IRQHandler

extern ASPEP_Handle_t aspepOverUartA;
extern MCP_Handle_t MCP_Over_UartA;
extern MCPA_Handle_t MCPA_UART_A;
#endif /* MCP_CONFIG_H */
