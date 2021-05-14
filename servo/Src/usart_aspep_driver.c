
#include <stdint.h>
#include "mc_stm_types.h"
#include "usart_aspep_driver.h"

void UASPEP_DAMCONFIG_TX (UASPEP_Handle_t *pHandle);
void UASPEP_DAMCONFIG_RX (UASPEP_Handle_t *pHandle);

void UASPEP_INIT (void *pHWHandle)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *) pHWHandle;
  UASPEP_DAMCONFIG_TX (pHandle);
  UASPEP_DAMCONFIG_RX (pHandle);
}

void UASPEP_DAMCONFIG_TX (UASPEP_Handle_t *pHandle)
{
/* Enable DMA UART */
 LL_USART_ClearFlag_TC (pHandle->USARTx);
 LL_USART_EnableIT_TC (pHandle->USARTx);

/* Enable DMA UART to start the TX request */
 LL_USART_EnableDMAReq_TX (pHandle->USARTx);

/* Write the USART_TDR register address in the DMA control register to configure it as
the destination of the transfer. */
  LL_DMA_SetPeriphAddress ( pHandle->txDMA, pHandle->txChannel , ( uint32_t ) &pHandle->USARTx->TDR );
 //  clear UART ISR
  LL_USART_ClearFlag_TC (pHandle->USARTx);

  /* DMA end of transfer on UART TX channel completion is not activated*/
  /* we prefer to activate UART TC itself to avoid to trig IT while queued data are still to be transmitted */
}

void UASPEP_DAMCONFIG_RX (UASPEP_Handle_t *pHandle)
{

 /* Enable DMA end of transfer on UART RX channel completion */
 LL_DMA_EnableIT_TC (pHandle->rxDMA, pHandle->rxChannel);

  /* Enable Error interrupt (EIE) to unmask Overrun interrupt */
 LL_USART_EnableIT_ERROR (pHandle->USARTx);

/* Write the USART_RDR register address in the DMA control register to configure it as
the source of the transfer. */
 LL_DMA_SetPeriphAddress ( pHandle->rxDMA, pHandle->rxChannel , ( uint32_t ) &pHandle->USARTx->RDR );

 //  clear UART ISR
 LL_USART_ClearFlag_TC (pHandle->USARTx);

 LL_USART_EnableDMAReq_RX (pHandle->USARTx);
}

bool UASPEP_SEND_PACKET (void *pHWHandle, void *data, uint16_t length)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *) pHWHandle;
  bool result;
  if ( LL_DMA_IsEnabledChannel (pHandle->txDMA, pHandle->txChannel) )
  {
    result = false;
  }
  else {
    LL_DMA_SetMemoryAddress( pHandle->txDMA, pHandle->txChannel, (uint32_t) data );
    LL_DMA_SetDataLength( pHandle->txDMA, pHandle->txChannel, length );
    LL_DMA_EnableChannel( pHandle->txDMA, pHandle->txChannel );
   result = true;
  }
  return result;
}

void UASPEP_RECEIVE_BUFFER (void *pHWHandle, void* buffer, uint16_t length)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *) pHWHandle;
  LL_DMA_DisableChannel( pHandle->rxDMA, pHandle->rxChannel );
  LL_DMA_SetMemoryAddress( pHandle->rxDMA, pHandle->rxChannel, (uint32_t) buffer );
  LL_DMA_SetDataLength(  pHandle->rxDMA, pHandle->rxChannel, length );
  LL_DMA_EnableChannel(  pHandle->rxDMA, pHandle->rxChannel );
}

void UASPEP_IDLE_ENABLE (void *pHWHandle)
{
   UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *) pHWHandle;
   LL_USART_ClearFlag_IDLE (pHandle->USARTx);
   LL_USART_EnableIT_IDLE (pHandle->USARTx);
}

