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
/*                      Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/

void STM32F0xx_Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,\
						GPIOMode_TypeDef Mode, GPIOSpeed_TypeDef Speed,\
						GPIOOType_TypeDef OType, GPIOPuPd_TypeDef PuPd)

uint8_t STM32F0xx_Pin_ReadBit(GPIO_TypeDef *GPIOx,uint16_t pin_no);

void STM32F0xx_Pin_WriteBit(GPIO_TypeDef *GPIOx,uint16_t pin_no, uint8_t val);

void STM32F2xx_Pin_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)

void STM32F0xx_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value);

/////////////interrupt APIs///////////////
/**
	* @brief  Configure the interrupt for a given pin number   
	* @param  pin_no : GPIO pin number 
	* @param  edge_sel   :  Triggering edge slection value of type "int_edge_sel_t"
	* @retval None
	*/
void STM32F0xx_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel);

/**
	* @brief  Enable the interrupt for a give pin number and irq number  
	* @param  pin_no : GPIO pin number 
	* @param  irq_no   :  irq_number to be enabled in NVIC 
	* @retval None
	*/
void STM32F0xx_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);

/**
	* @brief  Clear the sticky interrupt pending bit if set 
	* @param  pin_no : GPIO pin number 
	* @retval None
	*/
void STM32F0xx_clear_interrupt(uint16_t pin);


#endif 