/******************************************************************************

_____________________________________________________________________________
 File                   : STM32F0xx_GPIO_Setup.c
 Brief		    		: GPIO Driver for ST32F0xx Devices.
 Author's Name   		: Pranav Sanwal
 Date of creation		: 29-09-2017
 Status  		   		: **to be tested**
 Version		   		: 0.1
 Hardware Used   		:
 Software Aspects       :
 Dependency of          : STM32F0xx_GPIO_Setup.h
 Microcontroller used	: STM32F0xx
_____________________________________________________________________________
********************************************************************************/
#include <stdint.h>
#include "STM32F0xx_GPIO_Setup.h"
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
			   GPIOSpeed_TypeDef Speed,GPIOOType_TypeDef OType, GPIOPuPd_TypeDef PuPd)
{
	if(GPIOx == GPIOA)
		RCC_GPIOA_CLK_ENABLE();
	else if(GPIOx == GPIOB)
		RCC_GPIOB_CLK_ENABLE();
	else if(GPIOx == GPIOC)
		RCC_GPIOC_CLK_ENABLE();
	else if(GPIOx == GPIOD)
		RCC_GPIOD_CLK_ENABLE();
	// else if(GPIOx == GPIOE)
	// 	RCC_GPIOE_CLK_ENABLE();
	else if(GPIOx == GPIOF)
		RCC_GPIOB_CLK_ENABLE();

	gpio_configure_pin_mode(GPIOx,GPIO_Pin,Mode);
	gpio_configure_pin_speed(GPIOx,GPIO_Pin, Speed);
	gpio_configure_pin_otype(GPIOx,GPIO_Pin, OType);
	gpio_configure_pin_pupd(GPIOx,GPIO_Pin,PuPd);

}

/**
	* @brief  Read a value from a  given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @retval uint8_t: Value read 
	*/
uint8_t gpio_read_bit(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
	
	uint8_t value ; 
	
	value = ((GPIOx->IDR >> pin_no ) & 0x00000001 );
	
	return value ;
}

/**
	* @brief  Read a value from a  given Port 
	* @param  *GPIOx : GPIO Port Base address
	* @retval uint16_t: Value read 
	*/
uint16_t gpio_read_port(GPIO_TypeDef *GPIOx)
{
	uint16_t value ;

	value = (GPIOx->IDR)& 0xFFFF;
	
	return value;
}

/**
	* @brief toggle a pin at a given port
	* @param *GPIOx : GPIO Port Base address
	* @parma *pin_no : GPIO pin number
	* @retval None
	*/

void gpio_toggle (GPIO_TypeDef* GPIOx, uint16_t pin_no)
{
	GPIOx->ODR ^= pin_no;
}

/**
	* @brief  Write a value to given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  value   : value to be written 
	* @retval None
	*/
void gpio_write_bit(GPIO_TypeDef *GPIOx,uint16_t pin_no, uint8_t val)
{
	if(val)
		GPIOx->BSRR |=  (1 << pin_no);
	else
		GPIOx->BSRR &=  ~(1 << (pin_no + 0x10));
	
}

/**
	* @brief  Write a value to given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  value   : value to be written 
	* @retval None
	*/
void gpio_write_port(GPIO_TypeDef *GPIOx,uint16_t val)
{
	GPIOx->ODR = val ;
}

/**
	* @brief  Set the alternate functionality for the given pin  
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  alt_fun_value   :  alternate function to be configured with 
	* @retval None
	*/
void gpio_set_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value)
{
  if ( pin_no  <= 7 )
	{
		GPIOx->AFR[0] |= (alt_fun_value << ((pin_no) * 4 ));
	}
	else
	{
		GPIOx->AFR[1] |= (alt_fun_value << ( ( pin_no % 8) * 4 ));
	}
}


/**
	* @brief  Configures the mode of a pin : input, output, alt or analog mode 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  mode   : mode to be configured
	* @retval None
	*/
static void gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, GPIOMode_TypeDef mode)
{
	 GPIOx->MODER   |= (mode << ( 2 * pin_no));
}



/**
	* @brief  Configures the speed of a pin 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  speed   : value of the speed 
	* @retval None
	*/
static void gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, GPIOSpeed_TypeDef speed)
{
	
	 GPIOx->OSPEEDR |= (speed << (2 * pin_no));
}

/**
	* @brief  Configures the output type of a pin  
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  op_type   : output type to be configured with 
	* @retval None
	*/
static void gpio_configure_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin_no, GPIOOType_TypeDef op_type)
{
	
	 GPIOx->OTYPER |= (op_type << pin_no);
	
}

/**
	* @brief  Activates the internall pull up or pull down resistors
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  pupd   : specifies which resistor to activate
	* @retval None
	*/
static void gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no,GPIOPuPd_TypeDef pupd)
{
	 GPIOx->PUPDR |= (pupd << (2 * pin_no));

}
	

/**
	* @brief  Configure the interrupt for a given pin number   
	* @param  pin_no : GPIO pin number 
	* @param  edge_sel   :  Triggering edge slection value of type "int_edge_sel_t"
	* @retval None
	*/
void gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel)
{
	
	if (edge_sel == INT_RISING_EDGE )
	{
		EXTI->RTSR |= 1 << pin_no;
	}	
	else if (edge_sel == INT_FALLING_EDGE)
	{
		EXTI->FTSR |= 1 << pin_no;
	}else if (edge_sel == INT_RISING_FALLING_EDGE )
	{
		EXTI->FTSR |= 1 << pin_no;
		EXTI->RTSR |= 1 << pin_no;
	}
	else
	{
		 ;//TODO
	}
}

/**
	* @brief  Enable the interrupt for a give pin number and irq number  
	* @param  pin_no : GPIO pin number 
	* @param  irq_no   :  irq_number to be enabled in NVIC 
	* @retval None
	*/
void gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no)
{
	EXTI->IMR |= 1 << pin_no;
	NVIC_EnableIRQ(irq_no);
}

/**
	* @brief  Clear the sticky interrupt pending bit if set 
	* @param  pin_no : GPIO pin number 
	* @retval None
	*/
void gpio_clear_interrupt(uint16_t pin)
{
	if(EXTI->PR & (1 << pin ))
	{
		EXTI->PR |= 1 << pin;
	}
	
}
