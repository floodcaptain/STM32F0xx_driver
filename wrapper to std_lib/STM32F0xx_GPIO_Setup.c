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
 Dependency of        : STM32F0xx_GPIO_Setup.h
 Microcontroller used	: STM32F0xx
_____________________________________________________________________________

******************************************************************************/

//*****************************************************************************
//! \Function 		: STM32F0xx_Pin_Init
//! \Description	: Initializes Specified GPIO Pin.
//! \param 			: "GPIOx" : Where x can be (A..G) to Select The GPIO Peripheral.
//!                 : "GPIO_Pin": Specifies The Port Bit to be Written.
//!						This parameter can be one of GPIO_Pin_x where x can be (0..15) @ref GPIO_pins_define.
//!					: "BitVal": Specifies The Value to be Written to The Selected Pin.
//!						This parameter can be one of the BitAction enum values:
//!							"Bit_RESET" : to Clear the port pin
//!							"Bit_SET" : to set the port pin
//! 				: "Mode" : Specifies The Mode Of Operation Of Output Pin @ref GPIOMode_TypeDef.
//!					: "Speed" : Specifies The Speed Of Operation Of Output Pin @ref GPIOSpeed_TypeDef.
//!					: "OType" :	Specifies the operating output type @ref GPIOOType_TypeDef.
//!					: "PuPd" : Specifies the operating Pull-up/Pull-down @ref GPIOPuPd_TypeDef.
//! \return 		: None.
//*****************************************************************************
void STM32F0xx_Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
						GPIOMode_TypeDef Mode, GPIOSpeed_TypeDef Speed,
						GPIOOType_TypeDef OType, GPIOPuPd_TypeDef PuPd)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//! \Enable Port RCC_APB2 configurations
	if(GPIOx == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	else if(GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	else if(GPIOx == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	else if(GPIOx == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	else if(GPIOx == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	else if(GPIOx == GPIOF)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	else if(GPIOx == GPIOG)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	else if(GPIOx == GPIOH)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	else if(GPIOx == GPIOI)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = Mode;
	GPIO_InitStructure.GPIO_Speed = Speed;
	GPIO_InitStructure.GPIO_OType = OType;
	GPIO_InitStructure.GPIO_PuPd = PuPd;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//*****************************************************************************
//! \Function 		: STM32F2xx_Pin_Init
//! \Description	: Initializes Specified GPIO Pin.
//! \param 			: "GPIOx" : Where x can be (A..G) to Select The GPIO Peripheral.
//!                 : "GPIO_Pin": Specifies The Port Bit to be Written.
//!						This parameter can be one of GPIO_Pin_x where x can be (0..15) @ref GPIO_pins_define.
//!					: "BitVal": Specifies The Value to be Written to The Selected Pin.
//!						This parameter can be one of the BitAction enum values:
//!							"Bit_RESET" : to Clear the port pin
//!							"Bit_SET" : to set the port pin
//! 				: "Mode" : Specifies The Mode Of Operation Of Output Pin @ref GPIOMode_TypeDef.
//!					: "Speed" : Specifies The Speed Of Operation Of Output Pin @ref GPIOSpeed_TypeDef.
//!					: "OType" :	Specifies the operating output type @ref GPIOOType_TypeDef.
//!					: "PuPd" : Specifies the operating Pull-up/Pull-down @ref GPIOPuPd_TypeDef.
//! \return 		: None.
//*****************************************************************************
uint8_t STM32F0xx_Pin_ReadBit(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{

}