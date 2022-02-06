/*
 * stm32f401xx.h
 *
 *  Created on: Jan 25, 2022
 *      Author: abdelazim
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


//ARM Cortex Mx Processor NVIC ISERx register Addresses
#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )

//ARM Cortex Mx Processor NVIC ICERx register Addresses
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  			((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

//ARM Cortex Mx Processor Priority Register Address Calculation
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

//ARM Cortex Mx Processor number of priority bits implemented in Priority Register
#define NO_PR_BITS_IMPLEMENTED  4

//base addresses of Flash and SRAM memories
#define FLASH_BASEADDR						    0x08000000U
#define SRAM_BASEADDR						    0x20000000U
#define ROM_BASEADDR						    0x1FFF0000U

//AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

//Base addresses of peripherals which are hanging on AHB1 bus
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define CRC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x3000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)

//Base addresses of peripherals which are hanging on APB1 bus
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

//Base addresses of peripherals which are hanging on APB2 bus
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)

//peripheral register definition structures...

//GPIO_Reg
typedef struct
{
	__vo uint32_t MODER;                  
	__vo uint32_t OTYPER;                  
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 
}GPIO_RegDef_t;

//RCC_Reg
typedef struct
{
  __vo uint32_t CR;            /*offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*offset: 0x04 */
  __vo uint32_t CFGR;          /*offset: 0x08 */
  __vo uint32_t CIR;           /*offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*offset: 0x14 */
  	  uint32_t  RESERVED0[2];     /*Reserved, 0x18-0x1C*/
  __vo uint32_t APB1RSTR;      /*offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*offset: 0x24 */
  	  uint32_t  RESERVED1[2];  /*Reserved, 0x28-0x2C*/
  __vo uint32_t AHB1ENR;       /*offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*offset: 0x34 */
  	  uint32_t  RESERVED2[2];  /*Reserved,0x38-3C*/
  __vo uint32_t APB1ENR;       /*Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*offset: 0x44 */
  	  uint32_t  RESERVED3[2];
  __vo uint32_t AHB1LPENR;
  __vo uint32_t AHB2LPENR;
  	  uint32_t RESERVED4[2];
  __vo uint32_t APB1LPENR;
  __vo uint32_t APB2LPENR;
  	  uint32_t  RESERVED5[2];
  __vo uint32_t BDCR;
  __vo uint32_t CSR;
  	  uint32_t  RESERVED6[2];
  __vo uint32_t SSCGR;
  __vo uint32_t PLLI2SCFGR;
	  uint32_t  RESERVED7;
  __vo uint32_t DCKCFGR;

} RCC_RegDef_t;

//EXTI_Reg
typedef struct
{
	__vo uint32_t IMR;    /*Address offset: 0x00 */
	__vo uint32_t EMR;    /*Address offset: 0x04 */
	__vo uint32_t RTSR;   /* offset: 0x08 */
	__vo uint32_t FTSR;   /*Address offset: 0x0C */
	__vo uint32_t SWIER;  /* Address offset: 0x10 */
	__vo uint32_t PR;     /*Address offset: 0x14 */

}EXTI_RegDef_t;

//SYSCFG_Reg
typedef struct
{
	__vo uint32_t MEMRMP;       /*Address offset: 0x00      */
	__vo uint32_t PMC;          /*Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*Address offset: 0x08-0x14 */
	__vo uint32_t CMPCR;        /*Address offset: 0x20      */
} SYSCFG_RegDef_t;

//USART_Reg
typedef struct
{
	__vo uint32_t SR;         /*Address offset: 0x00 */
	__vo uint32_t DR;         /*Address offset: 0x04 */
	__vo uint32_t BRR;        /*Address offset: 0x08 */
	__vo uint32_t CR1;        /*Address offset: 0x0C */
	__vo uint32_t CR2;        /*Address offset: 0x10 */
	__vo uint32_t CR3;        /*Address offset: 0x14 */
	__vo uint32_t GTPR;       /*Address offset: 0x18 */
} USART_RegDef_t;

//type-casting Peripheral base addresses
#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)

//Clock Enable Macros for GPIOx peripherals
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

//Clock Enable Macros for USARTx peripherals
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))

//Clock Enable Macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))

//Clock Disable Macros for GPIOx peripherals/////////////////////////////////////////////////////////NOTE
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0))

/*
 * etc
 */

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)




//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET

//Bit position definitions USART_CR1
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

//Bit position definitions USART_CR2
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

//Bit position definitions USART_CR3
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

//Bit position definitions USART_SR
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include <stm32f401xx_gpio_driver.h>
#include <stm32f401xx_usart_driver.h>

#endif /* INC_STM32F401XX_H_ */
