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

static uint32_t _readResolution  = 10u;
static uint32_t _writeResolution = 8u;

void analogReadResolution(uint32_t res)
{
    _readResolution = res;
}

void analogWriteResolution(uint32_t res)
{
    _writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	return (from == to ? value : \
	            (from > to ? value >> (from-to) : value << (to-from)));
}

uint32_t analogRead(uint8_t ulPin)
{
	uint32_t ulValue = 0u;

	if(!(A(0) <= ulPin <= A(3)))//if(!(A0 <= ulPin <= A3))
		return (0u);

//	pinMode(ulPin, AN_INPUT);

#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
	ADC_RegularChannelConfig(ADC1, pInfo[ulPin].ulADCChannelNum, 1, ADC_SampleTime_55Cycles5);  /* ADC conversion time::STM32F103xx performance line devices: 1��s at 56MHz (1.17��s at 72MHz). The total conversion time is calculated as follows: Tconv = Sampling time + 12.5cycles */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
#elif defined(STM32F40_41xxx)
	ADC_RegularChannelConfig(ADC1, pInfo[ulPin].ulADCChannelNum, 1, ADC_SampleTime_15Cycles);
	ADC_SoftwareStartConv(ADC1);
#endif
	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	
	ulValue = ADC_GetConversionValue(ADC1);
	ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);
	
	return (ulValue);	
}
	 
void analogWrite(uint8_t ulPin, uint32_t ulValue)
{
	uint16_t TIM_CCR, dutycycle;
	
	if(ulPin >= PINS_COUNT)
		if(ulValue > 255u)
			return;
	
	if(pInfo[ulPin].ulTimerPeripheral == NULL)
	{
//		//Defaults to digital write
//		pinMode(ulPin, OUTPUT);
//		ulValue = mapResolution(ulValue, _writeResolution, 8u);
//
//		if(ulValue < 128u)
//			digitalWrite(ulPin, LOW);
//		else
//			digitalWrite(ulPin, HIGH);
		
		return;
	}
	
//	pinMode(ulPin, OUTPUT_AF_PP);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	ulValue = mapResolution(ulValue, _writeResolution, PWM_RESOLUTION);
	dutycycle = (ulValue * 100) / ((1 << _writeResolution) - 1);
	TIM_CCR = dutycycle * (TIM_ARR + 1) / 100;
    gTIM_OCInitStructure.TIM_Pulse = TIM_CCR;

	switch(pInfo[ulPin].ulTimerChannelNum)
	{
		case TIM_Channel_1:
			TIM_OC1Init(pInfo[ulPin].ulTimerPeripheral, &gTIM_OCInitStructure);
		break;

		case TIM_Channel_2:
			TIM_OC2Init(pInfo[ulPin].ulTimerPeripheral, &gTIM_OCInitStructure);
		break;
				
		case TIM_Channel_3:
			TIM_OC3Init(pInfo[ulPin].ulTimerPeripheral, &gTIM_OCInitStructure);
		break;
				
		case TIM_Channel_4:
			TIM_OC4Init(pInfo[ulPin].ulTimerPeripheral, &gTIM_OCInitStructure);
		break;
	}

//	TIM_Cmd(pInfo[ulPin].spTIMx_TimerPeripheral, ENABLE);
}
	 
#ifdef __cplusplus
 }
#endif
