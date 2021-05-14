#include "stdint.h"
#include "mcp.h"
#include "register_interface.h"
#include "mc_config.h"

void MCP_ReceivedPacket(MCP_Handle_t * pHandle)
{
  uint16_t * packetHeader;
  uint16_t command;
  uint8_t motorID;
  uint8_t MCPResponse;
  
  packetHeader = (uint16_t *) pHandle->rxBuffer;
  
  command = *packetHeader & CMD_MASK;
  motorID = (*packetHeader & MOTOR_MASK)-1;
  
  MC_Handle_t * pMCI = MC_Core_GetMotorControlHandle(motorID);
  /* Initialization of the tx length, command which send back data has to increment the txLength (case of Read register)*/
  pHandle->txLength = 0;
  
  switch (command) 
    {
    case SET_DATA_ELEMENT:
      MCPResponse = RI_SetRegCommandParser (pHandle); 
    break;
    case GET_DATA_ELEMENT:
      MCPResponse = RI_GetRegCommandParser (pHandle); 
      break;
    case START_MOTOR:
    {
	  MC_Status_t mc_status = MC_Core_GetStatus( pMCI );
	  if ( mc_status == MC_IDLE || mc_status == MC_STOP )
	  {
	    (void) MC_Core_Start( pMCI );
	    MCPResponse = MCP_CMD_OK;
	  }
	  else
	  {
		  MCPResponse = MCP_CMD_NOK;
	  }
    }
      break;
    case STOP_MOTOR: /* Todo: Check the relevance of return value*/
    case STOP_RAMP:
    {
      MC_Status_t mc_status = MC_Core_GetStatus( pMCI );
      if ( mc_status == MC_RUN )
      {
        (void) MC_Core_Stop( pMCI );
      }
      MCPResponse = MCP_CMD_OK;
    }
      break;
    case START_STOP:
      {
          MC_Status_t mc_status = MC_Core_GetStatus( pMCI );
          if ( mc_status == MC_IDLE || mc_status == MC_STOP )
          {
            (void) MC_Core_Start( pMCI );
          }
          else if ( mc_status == MC_RUN )
          {
            (void) MC_Core_Stop( pMCI );
          }
          MCPResponse = MCP_CMD_OK;
      }
      break;
    case FAULT_ACK:
      {
          MC_Status_t mc_status = MC_Core_GetStatus( pMCI );
          if ( mc_status == MC_SPEEDFBKERROR || mc_status == MC_OVERCURRENT || mc_status == MC_VALIDATION_FAILURE ||
               mc_status == MC_VALIDATION_BEMF_FAILURE || mc_status == MC_VALIDATION_HALL_FAILURE ||
    		   mc_status == MC_ADC_CALLBACK_FAILURE || mc_status == MC_LF_TIMER_FAILURE || MC_RUN_WRONG_STEP_FAILURE)
          {
            /* This call transitions the state to MC_STOP. Not an error state anymore and ready to start again. */
            (void) MC_Core_Stop( pMCI );
          }
        MCPResponse = MCP_CMD_OK;
      }
      break;
    case ENCODER_ALIGN:
    case IQDREF_CLEAR:
    	/* Do nothing at the moment */
    	MCPResponse = MCP_CMD_NOK;
      break;
    case PFC_ENABLE:
    case PFC_DISABLE:
    case PFC_FAULT_ACK:
    default :
      MCPResponse = MCP_CMD_UNKNOWN;
    }
    pHandle->txBuffer[pHandle->txLength] = MCPResponse;
    pHandle->txLength++;
}
