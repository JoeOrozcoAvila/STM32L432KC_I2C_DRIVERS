/*
 * stm32l432kc_i2c_driver.c
 *
 *  Created on: Dec 4, 2025
 *      Author: avila
 */
#include "stm32l432kc_i2c_driver.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScalar[4] = {2,4,8,16};

uint32_t getPLLClk(void)
{
	return 0;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DIS();
		}
		if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
	}
}
/* Generic steps for I2C Init:
 * 1.- Configure the Mode (standard or fast)
 * 2.- Configure the speed of the serial clock SCL
 * 3.- Configure the device address (applicable when device is slave)
 * 4.- Enable the acking (stm32l432kc dont have ACK)
 * 5.- Configure the rise time for I2C pins
 * */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// Using I2C Init Flowchart
	/* Clear PE bit in I2C_CR1
	 * Configure ANFOFF and DNF[3:0] in I2C_CR1
	 * Configure PRESC[3:0], SDADEL[3:0], SCLDEL[3:0], SCLH[7:0], SCLL[7:0], in I2C_TIMINGR
	 * Configure NOSTRETCH in I2C_CR1
	 * Set PE in I2C_CR1
	 * */
	// Reset I2C Peripheral
	I2C_PeripheralControl(pI2CHandle->pI2Cx, 0);

	// Configure TIMINGR registers
	pI2CHandle->pI2Cx->I2C_TIMINGR |= (pI2CHandle->I2C_Config.I2C_TimingrPRESC << 28);
	pI2CHandle->pI2Cx->I2C_TIMINGR |= (pI2CHandle->I2C_Config.I2C_TimingrSCLL << 0);
	pI2CHandle->pI2Cx->I2C_TIMINGR |= (pI2CHandle->I2C_Config.I2C_TimingrSCLH << 8);
	pI2CHandle->pI2Cx->I2C_TIMINGR |= (pI2CHandle->I2C_Config.I2C_TimingrSDADEL << 16);
	pI2CHandle->pI2Cx->I2C_TIMINGR |= (pI2CHandle->I2C_Config.I2C_TimingrSCLDEL << 20);

	// Configure the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; // to not touch OA1 reg in I2C_OAR1 that is bit 0
	pI2CHandle->pI2Cx->I2C_OAR1 |= tempreg;

}

void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{

}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);    // we need bits 2 and 3 from the CFGR in order to know which is the source clock
	if(clksrc == 0)
	{	// MSI configurable, default ==4MHz
		SystemClk = 4000000;
	}
	else if(clksrc == 1)
	{	// HSI16
		SystemClk = 16000000;
	}
	else if(clksrc == 2)
	{	// HSE -> 4MHz to 48MHz
		SystemClk = 4000000;
	}
	else if(clksrc == 3)
	{
		SystemClk = getPLLClk();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);  // check the prescalar that divides the clk reading bits 4 to 7 in CFGR

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScalar[temp-8];
	}

	// check the prescalar for APB1
	temp = ((RCC->CFGR >> 8) & 0x7);  // check the prescalar that divides the clk reading bits 8 to 10 in CFGR

		if(temp < 4)
		{
			apb1p = 1;
		}
		else
		{
			apb1p = APB1_PreScalar[temp-4];
		}

		pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}
