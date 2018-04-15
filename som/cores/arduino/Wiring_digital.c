/*
  Copyright (c) 2016 hathatehack  All right reserved.
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  Select the specified  port pin Mode and Init.
  * @param  PinNum: where PinNum can be (0..29, A0..A3) to select.
  * @param  PinMode:  specifies the Mode to select.
  * @retval None.
  */
void pinMode(uint8_t PinNum, uint32_t PinMode)
{
	GPIO_TypeDef       *GPIOx;
	uint16_t            GPIO_Pin;
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	if(PinNum >= PINS_COUNT)
		return;
	
	GPIOx    = pInfo[PinNum].ulPort;
	GPIO_Pin = pInfo[PinNum].ulPin;
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
	RCC_APB2PeriphClockCmd(pInfo[PinNum].ulPeripheral, ENABLE);
#elif defined(STM32F40_41xxx)
	RCC_AHB1PeriphClockCmd(pInfo[PinNum].ulPeripheral, ENABLE);
#endif
	
	switch(PinMode)
	{
		case OUTPUT:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
#endif
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        break;
		case OUTPUT_AF_PP:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
#endif
        break;
		case INPUT:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif
        break;
		case INPUT_PU:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
#endif
        break;
		case INPUT_PD:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
#endif
        break;
		case OUTPUT_AF_OD:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        break;
		case OUTPUT_OD:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        break;
		case INPUT_AN:
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
#elif defined(STM32F40_41xxx)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
#endif
        break;
        default:
        return;
	}
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
  * @brief  Sets/Reset the selected data port bits.
  * @param  PinNum: where PinNum can be (0..29, A0..A3) to select.
  * @param  PinStatus: specifies the port bits to be written.
  * @retval None
  */
void digitalWrite(uint8_t PinNum, uint8_t PinStatus)
{
	if(PinNum >= NUM_DIGITAL_PINS)
		return;
	
	if(PinStatus == HIGH)
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        pInfo[PinNum].ulPort->BSRR = pInfo[PinNum].ulPin;
#elif defined(STM32F40_41xxx)
	    pInfo[PinNum].ulPort->BSRRL= pInfo[PinNum].ulPin;
#endif
    else
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
        pInfo[PinNum].ulPort->BRR = pInfo[PinNum].ulPin;
#elif defined(STM32F40_41xxx)
	    pInfo[PinNum].ulPort->BSRRH= pInfo[PinNum].ulPin;
#endif
}

/**
  * @brief  Reads the specified input port pin.
  * @param  PinNum: where PinNum can be (0..29, A0..A3) to select.
  * @param  GPIOPin:  specifies the port bit to read.
  * @retval The input port pin value.
  */
//Unknow GPIO_ReadInputDataBit retval Bit_RESET == LOW and Bit_SET == HIGH
int digitalRead(uint8_t PinNum)
{
	if(PinNum >= NUM_DIGITAL_PINS)
		return (LOW);

	return (pInfo[PinNum].ulPort->IDR & pInfo[PinNum].ulPin ? HIGH : LOW);
}

#ifdef __cplusplus
}
#endif
