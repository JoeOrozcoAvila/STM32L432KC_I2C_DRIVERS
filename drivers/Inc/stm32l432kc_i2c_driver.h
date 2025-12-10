/*
 * stm32l432kc_i2c_driver.h
 *
 *  Created on: 21 nov 2025
 *      Author: avila
 *
 *  Driver implementation steps:
 *
 *  1.- Create stm32l432kc_i2c_driver.c and stm32l432kc_i2c_driver.h
 *  2.- Add I2C related details to mcu specific header file (stm32l432kc.h, taken from 37.7.12 I2C Register Map)
 *  	2.1. 	I2C peripheral register definition structure
 *  	2.2. 	I2C base address macros
 *  	2.3.	I2C peripheral definition macros
 *  	2.4. 	Macros to enable and disable I2C peripheral clock
 *  	2.5. 	Bit position definitions of I2C peripherals
 *  3.- Create handle structure and configuration structure in stm32l432kc_i2c_driver.h
 *  	3.1.	Define configuration parameters for config structure members
 *  4.- Create I2C API prototypes in stm32l432kc.h
 */

#ifndef INC_STM32L432KC_I2C_DRIVER_H_
#define INC_STM32L432KC_I2C_DRIVER_H_

#include "stm32l432kc.h"

/* STEP 3.
 * Configuration structure for I2Cx Peripheral (STEP 3.)
 * */
typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;			// Slave address is a 7 bit value given by the user
	//uint8_t		I2C_ACKControl;			// Removed due to in l432kc there is not an ACK control, it is managed dinamically by CR2, ISR and ICR bits 15, 4 and 4 respectivelly
	//uint16_t	I2C_FMDutyCycle;            // Removed due to there is no duty cycle bit
	uint8_t		I2C_TimingrSCLL;			// SCL Low Period (master mode)
	uint8_t		I2C_TimingrSCLH;     		// SCL High Period (master mode)
	uint8_t		I2C_TimingrSDADEL;			// SDA Data hold time (used to generate the delay tsdadel between SCL falling edge and SDA edge)
	uint8_t		I2C_TimingrSCLDEL;			// SCL Data hold time (used to generate a delay tscldel between SDA edge and SCL rising edge)
	uint8_t		I2C_TimingrPRESC;			// Timing prescaler, used to prescale I2CCLK in order to generate the clock period tpresc used for data setup and hold counters
}I2C_Config_t;
//think how to create a function that configures the TIMINGR register
/*
 * Handle structure for I2Cx peripheral
 * */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/* 3.1
 * Serial clock speed	*/
#define I2C_SCL_SPEED_SM		100000		// standard mode speed up to 100 Khz
#define I2C_SCL_SPEED_FM4K		400000      // fast mode speed up to 400 Khz
#define I2C_SCL_SPEED_FMPL		100000000   // fast mode plus up to 1 Mhz
//
/*
 * Peripheral control APIs
 * */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQPriority);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);



#endif /* INC_STM32L432KC_I2C_DRIVER_H_ */
