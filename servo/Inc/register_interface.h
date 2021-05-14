#ifndef REGISTER_INTERFACE_H
#define REGISTER_INTERFACE_H
#include "mcp.h"

/*
   MCP_ID definition :
   | Element Identifier 10 bits  |  Type  | Motor #|
   |                             |        |        |
   |15|14|13|12|11|10|09|08|07|06|05|04|03|02|01|00|

   Type definition :
   0	Reserved
   1	8-bit data
   2	16-bit data
   3	32-bit data
   4	Character string
   5	Raw Structure
   6	Reserved
   7	Reserved
*/
#define MCP_ID_SIZE 2 /* Number of byte */
#define ELT_IDENTIFIER_POS 6
#define TYPE_POS 3
#define TYPE_MASK 0x38
#define MOTOR_MASK 0x7
#define REG_MASK 0xFFF8

#define MOTORID(dataID) ((dataID & MOTOR_MASK)-1)

#define TYPE_DATA_SEG_END (0 << TYPE_POS)
#define TYPE_DATA_8BIT    (1 << TYPE_POS)
#define TYPE_DATA_16BIT   (2 << TYPE_POS)
#define TYPE_DATA_32BIT   (3 << TYPE_POS)
#define TYPE_DATA_STRING  (4 << TYPE_POS)
#define TYPE_DATA_RAW     (5 << TYPE_POS)
#define TYPE_DATA_FLAG    (6 << TYPE_POS)
#define TYPE_DATA_SEG_BEG (7 << TYPE_POS)

/* TYPE_DATA_8BIT registers definition */

#define  MC_REG_STATUS                 ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_CONTROL_MODE           ((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_RUC_STAGE_NBR          ((3  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_PFC_STATUS             ((13 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_PFC_ENABLED            ((14 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_SC_CHECK               ((15 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_SC_STATE               ((16 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_SC_STEPS               ((17 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_SC_PP                  ((18 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_SC_FOC_REP_RATE        ((19 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_SC_COMPLETED           ((20 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_POSITION_CTRL_STATE    ((21 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define  MC_REG_POSITION_ALIGN_STATE   ((22 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )

/* TYPE_DATA_16BIT registers definition */
#define  MC_REG_SPEED_KP               ((2   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_SPEED_KI               ((3   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_SPEED_KD               ((4   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_KP                 ((6   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_KI                 ((7   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_KD                 ((8   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_KP                 ((10  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_KI                 ((11  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_KD                 ((12  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_C1              ((13  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_C2              ((14  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_C1           ((15  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_C2           ((16  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_KI              ((17  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_KP              ((18  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FLUXWK_KP              ((19  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FLUXWK_KI              ((20  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FLUXWK_BUS             ((21  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_BUS_VOLTAGE            ((22  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_HEATS_TEMP             ((23  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_MOTOR_POWER            ((24  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_DAC_OUT1               ((25  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_DAC_OUT2               ((26  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_DAC_OUT3               ((27  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FLUXWK_BUS_MEAS        ((30  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_A                    ((31  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_B                    ((32  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_ALPHA                ((33  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_BETA                 ((34  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q                    ((35  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D                    ((36  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_REF                ((37  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_REF                ((38  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_V_Q                    ((39  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_V_D                    ((40  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_V_ALPHA                ((41  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_V_BETA                 ((42  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_ENCODER_EL_ANGLE       ((43  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_ENCODER_SPEED          ((44  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_EL_ANGLE        ((45  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_ROT_SPEED       ((46  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_I_ALPHA         ((47  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_I_BETA          ((48  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_BEMF_ALPHA      ((49  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_BEMF_BETA       ((50  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_EL_ANGLE     ((51  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_ROT_SPEED    ((52  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_I_ALPHA      ((53  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_I_BETA       ((54  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_BEMF_ALPHA   ((55  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOCORDIC_BEMF_BETA    ((56  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_DAC_USER1              ((57  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_DAC_USER2              ((58  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_HALL_EL_ANGLE          ((59  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_HALL_SPEED             ((60  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FF_VQ                  ((62  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FF_VD                  ((63  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FF_VQ_PIOUT            ((64  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FF_VD_PIOUT            ((65  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_DCBUS_REF          ((66  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_DCBUS_MEAS         ((67  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_ACBUS_FREQ         ((68  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_ACBUS_RMS          ((69  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_I_KP               ((70  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_I_KI               ((71  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_I_KD               ((72  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_V_KP               ((73  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_V_KI               ((74  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_V_KD               ((75  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_STARTUP_DURATION   ((76  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_SC_PWM_FREQUENCY       ((77  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_POSITION_KP            ((78  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_POSITION_KI            ((79  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_POSITION_KD            ((80  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_SPEED_KP_DIV           ((81  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_SPEED_KI_DIV           ((82  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_SPEED_KD_DIV           ((83  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_KP_DIV             ((84  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_KI_DIV             ((85  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_D_KD_DIV             ((86  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_KP_DIV             ((87  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_KI_DIV             ((88  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_I_Q_KD_DIV             ((89  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_POSITION_KP_DIV        ((90  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_POSITION_KI_DIV        ((91  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_POSITION_KD_DIV        ((92  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_I_KP_DIV           ((93  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_I_KI_DIV           ((94  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_I_KD_DIV           ((95  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_V_KP_DIV           ((96  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_V_KI_DIV           ((97  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PFC_V_KD_DIV           ((98  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_KI_DIV          ((99  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STOPLL_KP_DIV          ((100 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FLUXWK_KP_DIV          ((101 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_FLUXWK_KI_DIV          ((102 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_STARTUP_CURRENT_REF    ((103 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define  MC_REG_PULSE_VALUE            ((104 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )

/* TYPE_DATA_32BIT registers definition */
#define  MC_REG_FAULTS_FLAGS           ((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SPEED_MEAS             ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SPEED_REF              ((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_STOPLL_EST_BEMF        ((3  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_STOPLL_OBS_BEMF        ((4  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_STOCORDIC_EST_BEMF     ((5  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_STOCORDIC_OBS_BEMF     ((6  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_FF_1Q                  ((7  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_FF_1D                  ((8 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_FF_2                   ((9 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT ) /* To be checked shifted by >> 16*/
#define  MC_REG_PFC_FAULTS             ((40 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_CURRENT_POSITION       ((41 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_RS                  ((91 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_LS                  ((92 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_KE                  ((93 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_VBUS                ((94 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_MEAS_NOMINALSPEED   ((95 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_CURRENT             ((96 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_SPDBANDWIDTH        ((97 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_LDLQRATIO           ((98 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_NOMINAL_SPEED       ((99 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_CURRBANDWIDTH       ((100 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_J                   ((101 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_F                   ((102 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_MAX_CURRENT         ((103 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_STARTUP_SPEED       ((104 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define  MC_REG_SC_STARTUP_ACC         ((105 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )

#define  MC_REG_FW_VERSION            ((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )
#define  MC_REG_CTRL_BOARD_NAME       ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )
#define  MC_REG_PWR_BOARD_NAME        ((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )
#define  MC_REG_MOTOR_NAME            ((3  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )

#define  MC_REG_GLOBAL_CFG            ((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_MOTOR_CFG             ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_FOCFW_CFG             ((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_SPEED_RAMP            ((5  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_TORQUE_RAMP           ((6  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_REVUP_DATA            ((7  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW ) /* Configure all steps*/
#define  MC_REG_CURRENT_REF           ((13  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_POSITION_RAMP         ((14  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ASYNC_UARTA           ((20 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ASYNC_UARTB           ((21 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ASYNC_STLNK           ((22 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )

uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle);
uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle);
uint8_t RI_GetPtrReg (uint16_t dataID, void ** dataPtr);
uint8_t RI_GetIDSize (uint16_t ID);

#endif /* REGISTER_INTERFACE_H */
