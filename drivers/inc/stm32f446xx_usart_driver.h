/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Aug 22, 2025
 *      Author: csmla
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_


#include "stm32f446xx.h"

/*
 * USART config structure
 */
typedef struct{
 uint32_t USART_BaudRate;			/* USART Baud rate possible values @USART_BAUD */
 uint8_t USART_Mode;				/* possible values @USART_MODE */
  uint8_t USART_DataSize;			/* Frame size values @USART_DATA_SIZE  */
 uint8_t USART_NoOfStopbits;		/* stop bits 1 or 2 @USART_STOP_BITS*/
 uint8_t USART_ParityControl;		/* Parity @USART_PARITY */
 uint8_t USART_HWFlowControl;		/* HW flow control possible values @HW_FLOW_CTRL */

}USART_Config_t;


/*
 * Handle structure for a USART
 */
typedef struct{
	USART_RegDef_t *pUSARTx;   			/* base address of USARTx (x -> 1,2,3,4,5,6) */
	USART_Config_t USART_Config;
	uint8_t *TxBuffer;					/* to store addr of txbuff */
	uint8_t *RxBuffer;					/* to store add of rxbuff */
	uint8_t TxState;					/* to store communication state. possible values @ USART_STATUS */
	uint8_t RxState;					/* to store communication state. possible values @ USART_STATUS */
	uint32_t RxLen;
	uint32_t TxLen;

}USART_Handle_t;


/*
 * @USART_BAUD
 */
#define USART_BAUD_9600						9600
#define USART_BAUD_230400		    		230400
#define USART_BAUD_115200					115200
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 * USART_DATA
 */
#define USART_DATA_8BITS			0
#define USART_DATA_9BITS			1

/*
 * USART_STOP_BITS
 */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5			3
#define USART_STOPBITS_0_5			1



/*
 * @USART_PARITY
 */
#define USART_PARITY_DISABLE		0
#define USART_PARITY_EN_ODD			1
#define USART_PARITY_EN_EVEN		2

/*
 * @USART_HW_FLOW_CTRL
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * @USART_MODE
 */
#define USART_MODE_ONLY_TX			0
#define USART_MODE_ONLY_RX			1
#define USART_MODE_TXRX				2



/*
 * USART related status flag definitions
 */
#define USART_FLAG_PE        ( 1 << USART_SR_PE )
#define USART_FLAG_FE        ( 1 << USART_SR_FE )
#define USART_FLAG_NF        ( 1 << USART_SR_NF)
#define USART_FLAG_ORE       ( 1 << USART_SR_ORE )
#define USART_FLAG_IDLE      ( 1 << USART_SR_IDLE )
#define USART_FLAG_RXNE      ( 1 << USART_SR_RXNE )
#define USART_FLAG_TC        ( 1 << USART_SR_TC )
#define USART_FLAG_TXE       ( 1 << USART_SR_TXE )
#define USART_FLAG_LBD       ( 1 << USART_SR_LBD )
#define USART_FLAG_CTS       ( 1 << USART_SR_CTS )




/*
 * USART application events macros
 */
#define USART_EV_TX_CMPLT         	0
#define USART_EV_RX_CMPLT         	1
#define USART_EV_CTS			  	2
#define USART_EV_IDLE			  	3
#define USART_EV_ERR_ORE			4
#define USART_EV_ERR_NE				5
#define USART_EV_ERR_FE				6



/*
 * @USART_STATUS
 * USART_Status macros
 */
#define USART_STATUS_READY			0
#define USART_STATUS_BUSY_TX		1
#define USART_STATUS_BUSY_RX		2





/*********************************************************************************
 * API supported by this Driver. For more information refer to the Function definitions
 **************************************************************************************/
/*
 * Peripheral clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Peripheral init/ deint
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Peripheral read/ write
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);


uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_CloseSendData(USART_Handle_t *pUSARTHandle);
void USART_CloseReceiveData(USART_Handle_t *pUSARTHandle);

/*
 * peripheral Interrupt
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


/*
 * Application call backs
 */
void USART_ApplicationEventCallback (USART_Handle_t *pUSARTHandle, uint8_t AppEv);

/*
 * peripheral Control
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

void USART_EnableDisableInterrupt(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);



#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
