/*
 * stm32f446xx_rtc_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: csmla
 */


#include "stm32f446xx_rtc_driver.h"

static void BackupDomain_Unlock(void);
static void BackupDomain_Lock(void);

static void RTC_Unlock(void);
static void RTC_Lock(void);

static void RTC_ClkConfig(RTC_Config_t *pRTCConfig);

static void RTC_EnterCalendrInitMode(void);
static void RTC_ExitCalendrInitMode(void);

static void RTC_SetTimeDate(RTC_Handle_t *pRTCHandle);



#define RTC_TIMEOUT_CYCLES   (1000000UL)  /* ~safety limit */



/* Decimal to BCD and BCD to decimal conversion */
#define DECTOBCD(x)				(((x)/10 << 4) | ((x)%10))
#define BCDTODEC(x)				(((x) & 0x0F) + (((x) >> 4) * 10))




/******************************************************************************
 * @fn			- RTC_GetFlagStatus
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-FLAG_RESET or FLAG_SET macros
 *
 * @note		-
 */
uint8_t  RTC_GetFlagStatus (uint8_t flagName){

	if(RTC ->ISR & flagName) return SET;
	else return RESET;

}




/******************************************************************************
 * @fn			- WUT config
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */

void RTC_WUTConfig(RTC_WUT_Config_t *pWUTConfig){

}


/******************************************************************************
 * @fn			- WUT config
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void RTC_GetTimeDate(RTC_Time_t* pRTCTime){

	uint32_t temp1 =0;
	uint8_t temp2 = 0;

	/* chk BYPass shadow reg */
	if(RTC->CR & (1 << RTC_CR_BYPSHAD)){
		/* will implement */
	}
	else{
		/* clear RSF bit in RTC-ISR reg and wait until RSF is set before reading the RTC_SSR, RTC_TR and RTC_DR registers.*/
		RTC ->ISR &= ~ (1 << RTC_ISR_RSF);
		while(!(RTC ->ISR & (1 << RTC_ISR_RSF)));

		/*read time */
		temp1 = RTC->TR;

		temp2 = (temp1 >> RTC_TR_SU3_0) & 0x7F;
		pRTCTime->Seconds = BCDTODEC(temp2);

		temp2 = (temp1 >> RTC_TR_MNU3_0) & 0x7F;
		pRTCTime->Minutes = BCDTODEC(temp2);

		temp2 = (temp1 >> RTC_TR_HU3_0) & 0x3F;
		pRTCTime->Hours = BCDTODEC(temp2);

		temp2 = (temp1 >> RTC_TR_AM_PM) & 0x01;
		pRTCTime->AMPM = BCDTODEC(temp2);

		/*read Date */
		temp1 = RTC->DR;

		temp2 = (temp1 >> RTC_DR_DU3_0) & 0x3F;
		pRTCTime->Day = BCDTODEC(temp2);

		temp2 = (temp1 >> RTC_DR_MU3_0) & 0x1F;
		pRTCTime->Month = BCDTODEC(temp2);

		temp2 = (temp1 >> RTC_DR_YU3_0) & 0xFF;
		pRTCTime->Year = BCDTODEC(temp2);

		temp2 = (temp1 >> RTC_DR_WDU2_0) & 0x07;
		pRTCTime->Weekday = BCDTODEC(temp2);



	}


}





/******************************************************************************
 * @fn			- RTC_Init
 *
 * @brief		-
 *
 * @param[in]	- RTC Handle base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-none
 *
 * @note		-
 */
void RTC_Init(RTC_Handle_t *pRTCHandle){

	/*  Enable write to RCC_BDCR, RTC registers */
	/* Need to set DBP(Disable Backup domain write Protection) bit in RCC_PWR reg */
	BackupDomain_Unlock();

	/*  Optionally Resets the Backup Domain. This clears the clk rtc clk source and RTC regs */
	if(pRTCHandle ->RTC_Config.Reset_BackupDomain_EnorDi){
		RCC->BDCR |= (1 << 16);
		RCC->BDCR &= ~(1 << 16);
	}
	/* RTC_clk configuration */
	RTC_ClkConfig(& pRTCHandle->RTC_Config);
#if 0
	/* RTC_calendar Initialization */
	RTC_CalendarInit(& pRTCHandle->RTC_Config, & pRTCHandle ->RTC_Time);

	if(pRTCHandle ->WakeUp_EnorDi){
		RTC_WUTConfig(& pRTCHandle ->RTC_Wut_config);
	}
	if(pRTCHandle ->AlaramA_EnorDi){
		/* will implement later*/
	}
	if(pRTCHandle ->AlaramB_EnorDi){
		/* will implement later*/
	}
#endif

	/*Disable RTC write protection */
	RTC_Unlock();

	/* Enter rtc initialization mode */
	RTC_EnterCalendrInitMode();
	/* program prescalers */
	/* Prescalers (Async in PREDIV_A [22:16], Sync in PREDIV_S [14:0]) */
	/* Note: First pgm the sync prescaler then async prescaler */
	RTC ->PRER = (pRTCHandle->RTC_Config.Sync_Prescaler & 0x7FFF);
	RTC ->PRER |= ((pRTCHandle->RTC_Config.Async_Prescaler & 0x7F) << 16);


	/* RTC_SetTimeDate */
	RTC_SetTimeDate(pRTCHandle);

	/* exit initialization mode */
	RTC_ExitCalendrInitMode();

	/* enable rtc write protection */
	RTC_Lock();

	/* enable backup domain write protection */
	BackupDomain_Lock();


}






/**********************************************************************************************************
 * 									Helper Fns
 ************************************************************************************************************/
/***************************************************************************************
 * RTC enter calendar initialization mode
 ******************************************************************************************/
static void RTC_EnterCalendrInitMode(void){

	/* Set INIT bit to 1 in the RTC_ISR register to enter initialization mode.*/
	RTC->ISR |= (1 << RTC_ISR_INIT);
	/* wait util the INITF flag is set */
	while(!(RTC->ISR & (1 << RTC_ISR_INITF)));

}




/***************************************************************************************
 * RTC enter calendar initialization mode
 ******************************************************************************************/
static void RTC_ExitCalendrInitMode(void){

	/* clear INIT in the RTC_ISR register to exit initialization mode.*/
	RTC->ISR &= ~(1 << RTC_ISR_INIT);

}




/***************************************************************************************
 * RTC time date initialization
 ******************************************************************************************/
static void RTC_SetTimeDate(RTC_Handle_t *pRTCHandle){

	/* Load the initial time and date values in the shadow registers (RTC_TR and RTC_DR) */
	uint32_t temp;

	/*Load Seconds, minutes,Hours and format in BCD format*/
	temp =0;
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Seconds) << RTC_TR_SU3_0);
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Minutes) << RTC_TR_MNU3_0);
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Hours) << RTC_TR_HU3_0);
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.AMPM) << RTC_TR_AM_PM);

	RTC->TR = temp;

	/*Load Date, Month,weekday and year in BCD format*/
	temp =0;
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Day) << RTC_DR_DU3_0);
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Month) << RTC_DR_MU3_0);
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Weekday) << RTC_DR_WDU2_0);
	temp |= (DECTOBCD(pRTCHandle->RTC_Time.Year) << RTC_DR_YU3_0);

	RTC->DR = temp;


	/* configure the time format (12 or 24 hours) through the FMT bit in the RTC_CR */
	RTC ->CR |= (pRTCHandle ->RTC_Config.hour_format << RTC_CR_FMT);


}



/***********************************************************
 * Back up domain unlock
 ***************************************************************/
static void BackupDomain_Unlock(void){

	/*Enable PWR peripheral clock*/
	PWR_PCLK_EN();

	/*Disable Backup Domain write protection DBP*/
	PWR->CR |= (1 << 8);

}





/***************************************************************
 * Back up domain unlock
 ****************************************************************/
static void BackupDomain_Lock(void){

	/*Enable Backup Domain write protection DBP*/
	PWR->CR &= ~(1 << 8);

}


/***********************************************************************************************
 * RTC unlock disables rtc write protection
 * Note: unlock the write protection on all the RTC registers
 * except for RTC_ISR[31:8], RTC_TAFCR, and RTC_BKPxR.
 *****************************************************************************************/
static void RTC_Unlock(void){

	/* write 0xCA followed by 0x53 into the RTC WPR reg to disable write protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;


}

/**********************************************************************************************
 * RTC lock enables rtc write protection
  ********************************************************************************************/
static void RTC_Lock(void){

	/* write 0xFF into the RTC WPR reg to enables write protection */
	RTC->WPR = 0xFF;

}


/******************************************************************************************************
 * RTC-ClkConfig
 *****************************************************************************************************/
static void RTC_ClkConfig(RTC_Config_t *pRTCConfig){

	/* Enable the selected RTC clock sorce */
	/* Note: The selected clk source must be enabeld and ready before being selected as RTC_CLK source */
	switch (pRTCConfig->Clock_Source){

		uint32_t temp;
		case RTC_CLKSRC_LSE:
			/* LSEON */
			RCC->BDCR |= (1 << 0);

			/* wait until the clk is ready or timeout */
			temp = RTC_TIMEOUT_CYCLES;
			while((!(RCC->BDCR & (1 << 1))) && temp--);

			break;

		case RTC_CLKSRC_LSI:
			/* LSION */
			RCC->CSR |= (1 << 0);

			/* wait until the clk is ready or timeout */
			temp = RTC_TIMEOUT_CYCLES;
			while((!(RCC->CSR & (1 << 1))) && temp--);

			break;

		case RTC_CLKSRC_HSE_DIV128:
			/* LSION */
			RCC->CR |= (1 << 16);

			/* wait until the clk is ready or timeout */
			temp = RTC_TIMEOUT_CYCLES;
			while((!(RCC->CR & (1 << 17))) && temp--);

			break;

		default: BackupDomain_Lock();
	}

	/* Program the selected clk as RTC clock using RTCSEL bits in BDCR*/
	if(pRTCConfig->Clock_Source == RTC_CLKSRC_LSE) RCC->BDCR |= (RTC_CLKSRC_LSE << 8);
	else if(pRTCConfig->Clock_Source == RTC_CLKSRC_LSI) RCC->BDCR |= (RTC_CLKSRC_LSI << 8);
	else if(pRTCConfig->Clock_Source == RTC_CLKSRC_HSE_DIV128) RCC->BDCR |= (RTC_CLKSRC_HSE_DIV128 << 8);

	/* Enable the RTC Clock */
	RCC->BDCR |= (1 << 15);

}





/*********************************************************************************************
 *  									Interrupt
********************************************************************************************* */


/******************************************************************************
 * @fn			- RTC_IRQInterruptConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void RTC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			/* configure ISER0 */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		if(IRQNumber >= 32 && IRQNumber < 64){
			/* configure ISER1 */
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 64 && IRQNumber < 96){
			/* configure ISER2 */
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 96 && IRQNumber < 128){
			/* configure ISER3 */
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));
		}
	}
	/* Clear IRQ */
	else{
		if(IRQNumber <= 31){
			/* configure ICER0 */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		if(IRQNumber >= 32 && IRQNumber < 64){
			/* configure ICER1 */
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 64 && IRQNumber < 96){
			/* configure ICER2 */
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 96 && IRQNumber < 128){
			/* configure ICER3 */
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}

	}


}

/******************************************************************************
 * @fn			- RTC_IRQPriorityConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void RTC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;  			/* ipr reg number offset address */
	uint8_t iprx_section = IRQNumber % 4;    /* position in iprx reg */

	uint8_t shift_position = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);  //only upper 4 bits are implemented in IPR
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_position);


}



/******************************************************************************
 * 						Call back Fns
 ******************************************************************************/
__weak void RTC_ApplicationEventCallback (RTC_Handle_t *pRTCHandle, uint8_t AppEv){

	/* This is a weak function. Application may override it */
}








