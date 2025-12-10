/*
 * stm32l432kc.h
 *
 *	MCU specific header file
 *
 *  Created on: Oct 14, 2021
 *      Author: avila
 */

#ifndef INC_STM32L432KC_H_
#define INC_STM32L432KC_H_

#include <stdint.h>
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses, taken from file DUI0553.pdf in NVIC section
 * */
#define NVIC_ISER0						((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1						((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2						((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3						((volatile uint32_t *)0xE000E10C)
/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 * */
#define NVIC_ICER0						((volatile uint32_t *)0xE000E180)
#define NVIC_ICER1						((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2						((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3						((volatile uint32_t *)0xE000E18C)
/*
 * ARM Cortex Mx Processor Priority Register Addresses
 * */
#define NVIC_PR_BASE_ADDRESS			((volatile uint32_t *)0xE000E400)

/*
 * Base address of Flash and SRAM memories
 * */
#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U		//size of 48Kb
#define SRAM2_BASEADDR					0x2000C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM 							SRAM1_BASEADDR

/*
 * AHBx and APBx Bus peripheral base addresses
 * */
#define PERIPH_BASE						0x40000000U
#define APB1_BASEADDR					PERIPH_BASE
#define APB2_BASEADDR					0x40010000U
#define AHB1_BASEADDR					0x40020000U
#define AHB2_BASEADDR					0x48000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * */
#define DMA1_BASEADDR					(AHB1_BASEADDR + 0x0000)
#define DMA2_BASEADDR					(AHB1_BASEADDR + 0x0400)
#define RCC_BASEADDR					(AHB1_BASEADDR + 0x1000)
#define FLASHR_BASEADDR					(AHB1_BASEADDR + 0x2000)
#define CRC_BASEADDR					(AHB1_BASEADDR + 0x3000)
#define TSC_BASEADDR					(AHB1_BASEADDR + 0x4000)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 * */
#define GPIOA_BASEADDR					(AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB2_BASEADDR + 0x0800)
#define GPIOH_BASEADDR					(AHB2_BASEADDR + 0x1C00)
#define ADC_BASEADDR					(AHB2_BASEADDR + 0x08040000)
#define AES_BASEADDR					(AHB2_BASEADDR + 0x08060000)
#define RNG_BASEADDR					(AHB2_BASEADDR + 0x08060800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * */
#define TIM2_BASEADDR					(APB1_BASEADDR + 0x0000)
#define TIM6_BASEADDR					(APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR					(APB1_BASEADDR + 0x1400)
#define LCD_BASEADDR					(APB1_BASEADDR + 0x2400)
#define RTC_BASEADDR					(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR					(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR					(APB1_BASEADDR + 0x3000)
#define SPI2_BASEADDR					(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR					(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1_BASEADDR + 0x4800)
#define I2C1_BASEADDR					(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1_BASEADDR + 0x5C00)
#define CRS_BASEADDR					(APB1_BASEADDR + 0x6000)
#define CAN1_BASEADDR					(APB1_BASEADDR + 0x6400)
#define USBFS_BASEADDR					(APB1_BASEADDR + 0x6800)
#define USBSRAM_BASEADDR				(APB1_BASEADDR + 0x6C00)
#define PWR_BASEADDR					(APB1_BASEADDR + 0x7000)
#define DAC1_BASEADDR					(APB1_BASEADDR + 0x7400)
#define OPAMP_BASEADDR					(APB1_BASEADDR + 0x7800)
#define LPTIM1_BASEADDR					(APB1_BASEADDR + 0x7C00)
#define LPUART1_BASEADDR				(APB1_BASEADDR + 0x8000)
#define I2C4_BASEADDR					(APB1_BASEADDR + 0x8400)
#define SWPMI1_BASEADDR					(APB1_BASEADDR + 0x8800)
#define LPTIM2_BASEADDR					(APB1_BASEADDR + 0x9400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * */
#define SYSCFG_BASEADDR					(APB2_BASEADDR + 0x0000)
#define VREFBUF_BASEADDR				(APB2_BASEADDR + 0x0030)
#define COMP_BASEADDR					(APB2_BASEADDR + 0x0200)
#define EXTI_BASEADDR					(APB2_BASEADDR + 0x0400)
#define FIREWALL_BASEADDR				(APB2_BASEADDR + 0x1C00)
#define TIM1_BASEADDR					(APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR					(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR					(APB2_BASEADDR + 0x3800)
#define TIM15_BASEADDR					(APB2_BASEADDR + 0x4000)
#define TIM16_BASEADDR					(APB2_BASEADDR + 0x4400)
#define SAI1_BASEADDR					(APB2_BASEADDR + 0x5400)
#define DFSDM1_BASEADDR					(APB2_BASEADDR + 0x6000)

/*******************************************************************************************************************
 * *********   Peripheral Register Definition Structures   *********************************************************
 * *****************************************************************************************************************/
typedef struct
{
	volatile uint32_t MODER;				//OFFSET 0x00
	volatile uint32_t OTYPER;				//OFFSET 0x04
	volatile uint32_t OSPEEDR;				//OFFSET 0x08
	volatile uint32_t PUPDR;				//OFFSET 0x0C
	volatile uint32_t IDR;					//OFFSET 0x10
	volatile uint32_t ODR;					//OFFSET 0x14
	volatile uint32_t BSRR;					//OFFSET 0x18
	volatile uint32_t LCKR;					//OFFSET 0x1C
	volatile uint32_t AFRL;					//OFFSET 0x20
	volatile uint32_t AFRH;					//OFFSET 0x24
	volatile uint32_t BRR;					//OFFSET 0x28
}GPIO_RegDef_t;


typedef struct
{
	volatile uint32_t CR;					//OFFSET 0x00
	volatile uint32_t ICSCR;				//OFFSET 0x04
	volatile uint32_t CFGR;					//OFFSET 0x08
	volatile uint32_t PLLCFGR;				//OFFSET 0x0C
	volatile uint32_t PLLSAI1CFGR;			//OFFSET 0x10
	volatile uint32_t CIER;					//OFFSET 0x18
	volatile uint32_t CIFR;					//OFFSET 0x1C
	volatile uint32_t CICR;					//OFFSET 0x20
	volatile uint32_t AHB1RSTR;				//OFFSET 0x28
	volatile uint32_t AHB2RSTR;				//OFFSET 0x2C
	volatile uint32_t AHB3RSTR;				//OFFSET 0x30
	volatile uint32_t APB1RSTR1;			//OFFSET 0x38
	volatile uint32_t APB1RSTR2;			//OFFSET 0x3C
	volatile uint32_t APB2RSTR;				//OFFSET 0x40
	volatile uint32_t AHB1ENR;				//OFFSET 0x48
	volatile uint32_t AHB2ENR;				//OFFSET 0x4C
	volatile uint32_t AHB3ENR;				//OFFSET 0x50
	volatile uint32_t APB1ENR1;				//OFFSET 0x58
	volatile uint32_t APB1ENR2;				//OFFSET 0x5C
	volatile uint32_t APB2ENR;				//OFFSET 0x60
	volatile uint32_t AHB1SMENR;			//OFFSET 0x68
	volatile uint32_t AHB2SMENR;			//OFFSET 0x6C
	volatile uint32_t AHB3SMENR;			//OFFSET 0x70
	volatile uint32_t APB1SMENR1;			//OFFSET 0x78
	volatile uint32_t APB1SMENR2;			//OFFSET 0x7C
	volatile uint32_t APB2SMENR;			//OFFSET 0x80
	volatile uint32_t CCIPR;				//OFFSET 0x88
	volatile uint32_t BDCR;					//OFFSET 0x90
	volatile uint32_t CSR;					//OFFSET 0x94
	volatile uint32_t CRRCR;				//OFFSET 0x98
	volatile uint32_t CCIPR2;				//OFFSET 0x9C
}RCC_RegDef_t;


/*
 * Peripheral definition for EXTI register
 * */
typedef struct
{
	volatile uint32_t IMR1;					// Interrupt Mask Register 1
	volatile uint32_t EMR1;					// Event Mask Register 1
	volatile uint32_t RTSR1;				// Rising Trigger Selection Register 1
	volatile uint32_t FTSR1;				// Falling Trigger Selection Register 1
	volatile uint32_t SWIER1;				// Software Interrupt Event register 1
	volatile uint32_t PR1;					// Pending Register 1
	volatile uint32_t IMR2;					// Interrupt Mask register 2
	volatile uint32_t EMR2;					// Event Mask Register 2
	volatile uint32_t RTSR2;				// Rising Trigger Selection Register 2
	volatile uint32_t FTSR2;				// Falling Trigger Selection Register 2
	volatile uint32_t SWIER2;				// Software Interrupt Event register 2
	volatile uint32_t PR2;					// Pending Register 2
}EXTI_RegDef_t;

/*
 * Peripheral definition for SYSCFG register
 * */
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t CFGR1;
	volatile uint32_t EXTICR[3];
	volatile uint32_t SCSR;
	volatile uint32_t CFGR2;
	volatile uint32_t SWPR;
	volatile uint32_t SKR;
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for I2C (STEP I. I2C Register Map)
 * */
typedef struct {
	volatile uint32_t I2C_CR1;
	volatile uint32_t I2C_CR2;
	volatile uint32_t I2C_OAR1;
	volatile uint32_t I2C_OAR2;
	volatile uint32_t I2C_TIMINGR;
	volatile uint32_t I2C_TIMEOUTR;
	volatile uint32_t I2C_ISR;
	volatile uint32_t I2C_ICR;
	volatile uint32_t I2C_PECR;
	volatile uint32_t I2C_RXDR;
	volatile uint32_t I2C_TXDR;
}I2C_RegDef_t;
/*
 * Peripheral definitions macros (peripheral base addresses typecasted to xxx_RegDef_t pointer structure type
 * */
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
//(STEP II. I2C Base Address Macros or Peripheral definition macros (also STEP III.)
#define I2C1							((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*)I2C3_BASEADDR)
#define I2C4							((I2C_RegDef_t*)I2C4_BASEADDR)

/*
 * Clock enable macros for GPIOx Peripherals
 * */
#define GPIOA_PCLK_EN()					(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->AHB2ENR |= (1 << 2))
#define GPIOH_PCLK_EN()					(RCC->AHB2ENR |= (1 << 7))


/*
 * Clock disable macros for GPIOx Peripherals
 * */
#define GPIOA_PCLK_DIS()				(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()				(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()				(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOH_PCLK_DIS()				(RCC->AHB2ENR &= ~(1 << 7))
/*
 * Clock enable macros for I2C Peripherals (STEP IV.)
 * */
#define I2C1_PCLK_EN()					(RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN()					(RCC->APB1ENR1 |= (1 << 23))
#define I2C4_PCLK_EN()					(RCC->APB1ENR2 |= (1 << 1))
/*
 * Clock disable macros for I2C Peripherals (STEP IV.)
 * */
#define I2C1_PCLK_DIS()					(RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DIS()					(RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DIS()					(RCC->APB1ENR1 &= ~(1 << 23))
#define I2C4_PCLK_DIS()					(RCC->APB1ENR2 &= ~(1 << 1))
/*
 * Clock enable and disable macros for SYSCFG
 * */
#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 0))
#define SYSCFG_PCLK_DIS()				(RCC->APB2ENR &= ~(1 << 0))


#define GPIO_BASEADDR_TO_CODE(x) 		((x == GPIOA) ? 0 :\
		                                 (x == GPIOB) ? 1 :\
		                                 (x == GPIOC) ? 2 :\
		                                 (x == GPIOH) ? 3 :0)

/*Macros to reset GPIOx peripherals*/
#define GPIOA_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<0)); (RCC->AHB2RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<1)); (RCC->AHB2RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<2)); (RCC->AHB2RSTR &= ~(1<<2)); }while(0)
#define GPIOH_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<7)); (RCC->AHB2RSTR &= ~(1<<7)); }while(0)

/* IRQ Numbers of STM32L432KC MCU */
#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40

/*Generic macros*/
#define ENABLE							1
#define DISABLE 						0
#define SET 							ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET

/********************************************
 * Bit position definition for I2C CR1 (STEP V)
 ********************************************/
#define I2C_CR1_PE						0
#define I2C_CR1_TXIE					1
#define I2C_CR1_RXIE					2
#define I2C_CR1_ADDRIE					3
#define I2C_CR1_NACKIE					4
#define I2C_CR1_NACKIE		4
#define I2C_CR1_STOPIE		5
#define I2C_CR1_TCIE		6
#define I2C_CR1_ERRIE		7
#define I2C_CR1_DNF			8
#define I2C_CR1_ANFOFF		12
#define I2C_CR1_TXDMAEN		14
#define I2C_CR1_RXDMAEN		15
#define I2C_CR1_SBC			16
#define I2C_CR1_NOSTRETCH	17
#define I2C_CR1_WUPEN		18
#define I2C_CR1_GCEN		19
#define I2C_CR1_SMBHEN		20
#define I2C_CR1_SMBDEN		21
#define I2C_CR1_ALERTEN		22
#define I2C_CR1_PECEN		23
//Bit position definition I2C CR2
#define I2C_CR2_SADD0		0
#define I2C_CR2_SADD1		1
#define I2C_CR2_SADD2		8
#define I2C_CR2_RD_WRN		10
#define I2C_CR2_ADD10		11
#define I2C_CR2_HEAD10R		12
#define I2C_CR2_START		13
#define I2C_CR2_STOP		14
#define I2C_CR2_NACK		15
#define I2C_CR2_NBYTES		16
#define I2C_CR2_RELOAD		24
#define I2C_CR2_AUTOEND		25
#define I2C_CR2_PECBYTE		26
//Bit position definition I2C TIMINGR (CCR
#define I2C_TIMINGR_SCLL	0
#define I2C_TIMINGR_SCLH	8
#define I2C_TIMINGR_SDADEL	16
#define I2C_TIMINGR_SCLDEL  20
#define I2C_TIMINGR_PRESC	28
//Bit position definition I2C ISR (some similar registers of SR1 and SR2 for course)
#define I2C_ISR_TXE			0
#define I2C_ISR_TXIS		1
#define I2C_ISR_RXNE		2
#define I2C_ISR_ADDR		3
#define I2C_ISR_NACKF		4
#define I2C_ISR_STOPF		5
#define I2C_ISR_TC			6
#define I2C_ISR_TCR			7
#define I2C_ISR_BERR		8
#define I2C_ISR_ARLO		9
#define I2C_ISR_OVR			10
#define I2C_ISR_PECERR		11
#define I2C_ISR_TIMEOUT		12
#define I2C_ISR_ALERT		13
#define I2C_ISR_BUSY		15
#define I2C_ISR_DIR			16
#define I2C_ISR_ADDCODE		17
//Bit position definition I2C ICR
#define I2C_ICR_ADDRCF		3
#define I2C_ICR_NACKCF		4
#define I2C_ICR_STOPCF		5
#define I2C_ICR_BERRCF		8
#define I2C_ICR_ARLOCF		9
#define I2C_ICR_OVRCF		10
#define I2C_ICR_PECCF		11
#define I2C_ICR_TIMOUTCF	12
#define I2C_ICR_ALERTCF		13


#include "stm32l432kc_gpio_driver.h"
#include "stm32l432kc_i2c_driver.h"

#endif /* INC_STM32L432KC_H_ */
