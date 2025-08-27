/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Aug 24, 2025
 *      Author: csmla
 */


#include "stm32f446xx_rcc_driver.h"

uint16_t ahb_PreScaler[10] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t abp1_PreScaler[4] = {2, 4, 8, 16};
uint8_t abp2_PreScaler[4] = {2, 4, 8, 16};


/******************************************************************************
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- (PCLK1 = system clk freq / AHB pre scaler) /APB1 pre scaler
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-uint32_t periperal clk1 value
 *
 * @note		-
 */
uint32_t  RCC_GetPCLK1Value (void){


	uint32_t  pclk1, systemClk;
	uint16_t ahbp;
	uint8_t clkSrc, temp, apb1p;

	clkSrc = (RCC -> CFGR >> 2) & 0x3;

	/* get system clk freq */
	if(clkSrc == 0) systemClk = HSI_CLK_FREQ;
	else if(clkSrc == 1) systemClk = HSE_CLK_FREQ;

	/* get ahb prescaler */
	temp = (RCC -> CFGR >> 4) & 0xF;

	if(temp < 8) ahbp = 1;
	else ahbp = ahb_PreScaler[temp - 8];

	/* get apb2 prescaler */
	temp = (RCC -> CFGR >> 13) & 0x7;

	if(temp < 4) apb1p = 1;
	else apb1p = abp1_PreScaler[temp - 4];

	pclk1 = (systemClk / ahbp /apb1p);

	return pclk1;


}


/******************************************************************************
 * @fn			- RCC_GetPCLK2Value
 *
 * @brief		- (PCLK1 = system clk freq / AHB pre scaler) /APB1 pre scaler
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-uint32_t periperal clk2 value
 *
 * @note		-
 */
uint32_t  RCC_GetPCLK2Value (void){


	uint32_t  pclk2, systemClk;
	uint16_t ahbp;
	uint8_t clkSrc, temp, apb2p;

	clkSrc = (RCC -> CFGR >> 2) & 0x3;

	if(clkSrc == 0) systemClk = HSI_CLK_FREQ;
	else if(clkSrc == 1) systemClk = HSE_CLK_FREQ;

	temp = (RCC -> CFGR >> 4) & 0xF;

	if(temp < 8) ahbp = 1;
	else ahbp = ahb_PreScaler[temp - 8];


	temp = (RCC -> CFGR >> 10) & 0x7;

	if(temp < 4) apb2p = 1;
	else apb2p = abp2_PreScaler[temp - 4];

	pclk2 = (systemClk / ahbp /apb2p);

	return pclk2;


}




