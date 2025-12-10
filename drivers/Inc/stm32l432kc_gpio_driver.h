/*
 * stm32l432kc_gpio_driver.h
 *
 *	Driver specific data header file
 *
 *  Created on: Oct 14, 2021
 *      Author: avila
 */
#ifndef INC_STM32L432KC_GPIO_DRIVER_H_
#define INC_STM32L432KC_GPIO_DRIVER_H_

#include "stm32l432kc.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_Pin_Config_t;



/*Handle structure for GPIO pin*/
typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t  *pGPIOx;
	//This holds the base address of the GPIO port to which the pin belongs
	GPIO_Pin_Config_t GPIO_PinConfig; //this variable holds GPIO pin configuration settings
}GPIO_Handle_t;

/*
 * GPIO pin number macros
 * */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * GPIO pin possible modes macros
 * */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// Not in the document, interrupt mode can be set connecting the gpio pin to EXTI
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * GPIO pin possible output type macros
 * */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * GPIO pin possible output speeds macros
 * */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO pin possible pull up and pull down macros
 * */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*******************************************************************************************************************
 * *********   APIs supported by this driver (function prototypes  *************************************************
 * *****************************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);  //original

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); //use the RCC peripheral reset register, just take the base address of the peripheral



uint8_t  GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32L432KC_GPIO_DRIVER_H_ */
