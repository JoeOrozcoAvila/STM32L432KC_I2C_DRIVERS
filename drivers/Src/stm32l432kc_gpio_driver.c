/*
 * stm32l432kc_gpio_driver.c
 *
 *  Created on: Oct 14, 2021
 *      Author: avila
 */
#include "stm32l432kc_gpio_driver.h"
/*
 * Function that enables or disables peripheral clock for the given GPIO port
 * Param1:	base address of the gpio peripheral
 * Param2:  ENABLE or DISABLE macros
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DIS();
			}
		else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DIS();
			}
		else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DIS();
			}
		else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DIS();
			}
	}
}
/*Function that initialize given GPIO port and GPIO pin
 * Param1:	base address of the gpio peripheral
 * Param2:  ENABLE or DISABLE macros*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		//                 pin mode: 1                  <<  2 *  (pin#4) to set pin 4 as output
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //store the value of temp into the actual register that is MODER
		temp = 0;
	}
	else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{   // Configure the FTSR1 or 2?
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR1 or 2?
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure the FTSR1 or 2?
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Configure the RTSR1 bit
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);
		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}
	//configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
		{	//pin 0 to 7 low register
			pGPIOHandle->pGPIOx->AFRL &= (0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp));
		}
		else
		{	//pin 8 to 15 high register
			pGPIOHandle->pGPIOx->AFRH &= (0xF << (4 * temp));//clearing
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp));
		}
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) //use the RCC peripheral reset register, just take the base address of the peripheral
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


uint8_t  GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				// Program ISER0 register
				*NVIC_ISER0 |= (1 << IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				// Program ISER1
				*NVIC_ISER1 |= (1 << (IRQNumber % 32));

			}
			else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				// Program ISER2
				*NVIC_ISER1 |= (1 << (IRQNumber % 64));
			}
		}
		else
		{
			if(IRQNumber <= 31)
			{
				// Program ICER0 register
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				// Program ICER1
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			}
			else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				// Program ICER2
				*NVIC_ICER1 |= (1 << (IRQNumber % 64));
			}
		}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Find the IPR Register
	uint8_t iprx = IRQNumber / 4;
	// Find the iprx_section of the register (each IPR register is divided in 4 blocks of 8 bits
	uint8_t iprx_section = IRQNumber / 4;
	*(NVIC_PR_BASE_ADDRESS + (iprx * 4)) |= (IRQPriority << ((8 * iprx_section) + 4)); // because need the high 4 bits
	// uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED)

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR1 & (1 << PinNumber)) // if the PR bitposition corresponding to this pinnumber is set then  the interrupt is pend
	{
		//clear
		EXTI->PR1 |= (1 << PinNumber);
	}
}
