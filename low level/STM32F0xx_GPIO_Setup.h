/******************************************************************************
_____________________________________________________________________________
 File                   : STM32F0xx_GPIO_Setup.h
 Brief		    		: GPIO Driver for ST32F0xx Devices.
 Author's Name   		: Pranav Sanwal
 Date of creation		: 29-09-2017
 Status  		   		: **to be tested**
 Version		   		: 0.1
 Hardware Used   		:
 Software Aspects       :
 Dependency of        : STM32F0xx_GPIO_Setup.h
 Microcontroller used	: STM32F0xx
 _____________________________________________________________________________

******************************************************************************/

//! \Define to prevent recursive inclusion
#ifndef _STM32F0xx_GPIO_DRIVER_H
#define _STM32F0xx_GPIO_DRIVER_H

#include "stm32f0xx.h"


/******************************************************************************/
/*                                                                            */
/*                      Macros used for GPIO pin Initialization               */
/*                                                                            */
/******************************************************************************/
/** @ref Configuration_Mode_enumeration 
  * @{
  */
typedef enum
{
  GPIO_Mode_IN   = (uint32_t) 0x00, /*!< GPIO Input Mode              */
  GPIO_Mode_OUT  = (uint32_t)0x01, /*!< GPIO Output Mode             */
  GPIO_Mode_AF   = (uint32_t)0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = (uint32_t)0x03  /*!< GPIO Analog In/Out Mode      */
}GPIOMode_TypeDef;
/**	
	*@}
	*/


/** @ref Output_type_enumeration
  * @{
  */
typedef enum
{
  GPIO_OType_PP = (uint32_t)0x00,
  GPIO_OType_OD = (uint32_t)0x01
}GPIOOType_TypeDef;
/**	
	*@}
	*/

/** @ref Output_type_enumeration
  * @{
  */
typedef enum
{
  GPIO_Speed_Low  = (uint32_t)0x00, /*!< I/O output speed: Low 2 MHz */
  GPIO_Speed_Medium  = (uint32_t)0x01, /*!< I/O output speed: Medium 10 MHz */
  GPIO_Speed_High  = (uint32_t)0x03  /*!< I/O output speed: High 50 MHz */
}GPIOSpeed_TypeDef;
/**	
	*@}
	*/


/** @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
typedef enum
{
  GPIO_PuPd_NOPULL = (uint32_t)0x00,
  GPIO_PuPd_UP     = (uint32_t)0x01,
  GPIO_PuPd_DOWN   = (uint32_t)0x02
}GPIOPuPd_TypeDef;
/**	
	*@}
	*/

/**
* @brief  Interrupt Edge selection enum 
*/
typedef enum
{
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;
/**	
	*@}
	*/

/* Macros to Enable Clock for diffrent GPIO ports in RCC register */

#define RCC_GPIOA_CLK_ENABLE()       (RCC->AHBENR |=  (RCC_AHBENR_GPIOAEN)  )
#define RCC_GPIOB_CLK_ENABLE()       (RCC->AHBENR |=  (RCC_AHBENR_GPIOBEN ) )
#define RCC_GPIOC_CLK_ENABLE()       (RCC->AHBENR |=  (RCC_AHBENR_GPIOCEN ) )
#define RCC_GPIOD_CLK_ENABLE()       (RCC->AHBENR |=  (RCC_AHBENR_GPIODEN ) )
//#define RCC_GPIOE_CLK_ENABLE()       (RCC->AHB1ENR |=  (RCC_AHBENR_GPIOEEN ) ) //not available on stm32f030
#define RCC_GPIOF_CLK_ENABLE()       (RCC->AHBENR |=  (RCC_AHBENR_GPIOFEN ) )


/******************************************************************************/
/*                                                                            */
/*                      Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/

/**
	* @brief  Initializes the gpio pin 
	* @param  *GPIOx : GPIO Port Base address
	* @param  GPIO_Pin :Pointer to the pin conf structure sent by application 
	* @param  Mode : Specifies The Mode Of Operation Of Output Pin @ref GPIO_Mode_Settings_values.
	* @param  Speed : Specifies The Speed Of Operation Of Output Pin @ref GPIO_Speed_type_selection_values.
	* @param  OType : Specifies the operating output type @ref GPIO_OP_type_selection_values .
	* @param  PuPd : Specifies the operating Pull-up/Pull-down @ref GPIO_pull_up/pull_dwn_selection_values .
	* @retval None
	*/
void gpio_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef Mode,\
			   GPIOSpeed_TypeDef Speed,GPIOOType_TypeDef OType, GPIOPuPd_TypeDef PuPd);


/**
	* @brief  Read a value from a  given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @retval uint8_t: Value read 
	*/
uint8_t gpio_read_bit(GPIO_TypeDef *GPIOx,uint16_t pin_no);

/**
	* @brief  Read a value from a  given Port 
	* @param  *GPIOx : GPIO Port Base address
	* @retval uint16_t: Value read 
	*/
uint16_t gpio_read_port(GPIO_TypeDef *GPIOx);

/**
	* @brief toggle a pin at a given port
	* @param *GPIOx : GPIO Port Base address
	* @parma *pin_no : GPIO pin number
	* @retval None
	*/

void gpio_toggle (GPIO_TypeDef* GPIOx, uint16_t pin_no);

/**
	* @brief  Write a value to given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  value   : value to be written 
	* @retval None
	*/
void gpio_write_bit(GPIO_TypeDef *GPIOx,uint16_t pin_no, uint8_t val);

/**
	* @brief  Write a value to given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  value   : value to be written 
	* @retval None
	*/
void gpio_write_port(GPIO_TypeDef *GPIOx,uint16_t val);

/**
	* @brief  Set the alternate functionality for the given pin  
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  alt_fun_value   :  alternate function to be configured with 
	* @retval None
	*/
void gpio_set_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value);

/**
	* @brief  Configure the interrupt for a given pin number   
	* @param  pin_no : GPIO pin number 
	* @param  edge_sel   :  Triggering edge slection value of type "int_edge_sel_t"
	* @retval None
	*/
void gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel);

/**
	* @brief  Enable the interrupt for a give pin number and irq number  
	* @param  pin_no : GPIO pin number 
	* @param  irq_no   :  irq_number to be enabled in NVIC 
	* @retval None
	*/
void gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);

/**
	* @brief  Clear the sticky interrupt pending bit if set 
	* @param  pin_no : GPIO pin number 
	* @retval None
	*/
void gpio_clear_interrupt(uint16_t pin);


#endif 