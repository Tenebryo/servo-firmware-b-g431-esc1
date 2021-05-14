#ifndef usart_aspep_driver_h
#define usart_aspep_driver_h

#include <stdint.h>
#include <stdbool.h>
  
/* To be removed no protocol awarness at this level */

typedef struct {
  USART_TypeDef *USARTx;
  DMA_TypeDef * rxDMA;
  DMA_TypeDef * txDMA;
  uint32_t rxChannel;
  uint32_t txChannel;
} UASPEP_Handle_t;

bool UASPEP_SEND_PACKET (void *pHandle, void *data, uint16_t length); 
void UASPEP_RECEIVE_BUFFER (void *pHandle, void* buffer, uint16_t length);
void UASPEP_INIT (void *pHandle);
void UASPEP_IDLE_ENABLE (void *pHWHandle);

#endif