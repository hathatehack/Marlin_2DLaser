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
#include "variant.h"

#ifdef __cplusplus
 extern "C" {
#endif
	
const PinInfo pInfo[] = {
        //-----------------------------InputPort-----------------------------//
 /*  0*/{GPIOA, GPIO_Pin_0,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},//stepper
        {GPIOA, GPIO_Pin_1,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},
        {GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},
        {GPIOC, GPIO_Pin_13, RCC_APB2Periph_GPIOC, NONE,          NULL, NONE},
        {GPIOC, GPIO_Pin_14, RCC_APB2Periph_GPIOC, NONE,          NULL, NONE},
        {GPIOC, GPIO_Pin_15, RCC_APB2Periph_GPIOC, NONE,          NULL, NONE},
        {GPIOB, GPIO_Pin_5,  RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},//oled
        {GPIOB, GPIO_Pin_6,  RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},
        {GPIOB, GPIO_Pin_7,  RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},
        {GPIOB, GPIO_Pin_8,  RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},
        {GPIOB, GPIO_Pin_9,  RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},
        {GPIOD, GPIO_Pin_2,  RCC_APB2Periph_GPIOD, NONE,          NULL, NONE},
 /* 12*/{GPIOC, GPIO_Pin_5,  RCC_APB2Periph_GPIOC, NONE,          NULL, NONE},//power
        //--------------------------------END--------------------------------//

        //---------------------------USB_To_Serial---------------------------//
 /*   */{GPIOA, GPIO_Pin_9,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //USART1_TX
        {GPIOA, GPIO_Pin_10, RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //USART1_RX
        {GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //USART2_TX
        {GPIOA, GPIO_Pin_3,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //USART2_RX
        //--------------------------------END--------------------------------//

        //--------------------------------SPI--------------------------------//
 /*   */{GPIOA, GPIO_Pin_4,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //SPI1_SS
        {GPIOA, GPIO_Pin_5,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //SPI1_CSK
        {GPIOA, GPIO_Pin_6,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //SPI1_MISO
        {GPIOA, GPIO_Pin_7,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //SPI1_MOSI
// /*   */{GPIOB, GPIO_Pin_12, RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},              //SPI2_SS
//        {GPIOB, GPIO_Pin_13, RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},              //SPI2_CSK
//        {GPIOB, GPIO_Pin_14, RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},              //SPI2_MISO
//        {GPIOB, GPIO_Pin_15, RCC_APB2Periph_GPIOB, NONE,          NULL, NONE},              //SPI2_MOSI
//        //--------------------------------END--------------------------------//

        //--------------------------------LEDs-------------------------------//
 /*   */{GPIOA, GPIO_Pin_5,  RCC_APB2Periph_GPIOA, NONE,          NULL, NONE},              //LED_RUN
        {GPIOC, GPIO_Pin_4,  RCC_APB2Periph_GPIOC, NONE,          NULL, NONE},              //LED_STATUS
        //--------------------------------END--------------------------------//

        //----------------------------Analog pins----------------------------//
        //PortA
 /*N-4*/{GPIOA, GPIO_Pin_0,  RCC_APB2Periph_GPIOA, ADC_Channel_0, NULL, NONE},              //ADC0
        {GPIOA, GPIO_Pin_1,  RCC_APB2Periph_GPIOA, ADC_Channel_1, NULL, NONE},              //ADC1
        {GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA, ADC_Channel_2, NULL, NONE},              //ADC2
        {GPIOA, GPIO_Pin_3,  RCC_APB2Periph_GPIOA, ADC_Channel_3, NULL, NONE},              //ADC3
        //--------------------------------END--------------------------------//
};

#ifdef __cplusplus
}
#endif

 // ----------------------------------------------------------------------------
 /*
  * USART objects
  */
//RingBuffer rx_buffer0;
//RingBuffer rx_buffer1;

//USARTClass Serial(USART1, USART1_IRQn, ID_USART0, &rx_buffer0);
//void serialEvent() __attribute__((weak));
//void serialEvent() {};
//USARTClass Serial1(USART2, USART2_IRQn, ID_USART1, &rx_buffer1);
//void serialEvent1() __attribute__((weak));
//void serialEvent1() {};

// IT handlers
//void USART1_IRQHandler(void)
//{
//	Serial.IrqHandler();
//}

//void USART2_IRQHandler(void)
//{
//	Serial1.IrqHandler();
//}

// ----------------------------------------------------------------------------
void serialEventRun(void)
{
//	if(Serial.available())
//		serialEvent();
//	if(Serial1.available())
//		serialEvent1();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

const uint8_t A0 = A(0);
const uint8_t A1 = A(1);
const uint8_t A2 = A(2);
const uint8_t A3 = A(3);

#define TIM_PRESCALER   (0u)
uint16_t TIM_ARR = ((SystemCoreClock) / (TIM_PRESCALER + 1u)) / (PWM_FREQUENCY) - 1u;
TIM_TimeBaseInitTypeDef gTIM_TimeBaseStructure;
TIM_OCInitTypeDef       gTIM_OCInitStructure;

void time_init(void);
void __libc_init_array(void);
void init(void)
{
    SystemInit();
    /* Configure the NVIC Preemption Priority Bits */
    /* 4 bits for pre-emption priority(0-15 PreemptionPriority) and 0 bits for subpriority(0 SubPriority) */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_SetPriority(SysTick_IRQn, 15);
    /* Set Systick to 1ms interval, common to all stm32f10x variants */
    if(SysTick_Config(SystemCoreClock / 1000u))
        while(1);

//  /* Initialize C library */
    __libc_init_array();

#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#if defined(STM32F10X_HD)
//  /* Remap */
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
//    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#endif
#endif

    pinMode(PIN_LED_RUN, OUTPUT);
    digitalWrite(PIN_LED_RUN, LOW);
//    pinMode(PIN_LED_STATUS, OUTPUT);
//    digitalWrite(PIN_LED_STATUS, HIGH);

    time_init();
}

void time_init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure = {0,};

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseStructure.TIM_Period        = 0xFFFF-1;
  TIM_TimeBaseStructure.TIM_Prescaler     = 36-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ(TIM2_IRQn);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseStructure.TIM_Period        = 1000-1;//40000-1;
  TIM_TimeBaseStructure.TIM_Prescaler     = 600;//7200-1;//72-1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_Pulse  = 999;//1;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
  NVIC_SetPriority(TIM3_IRQn, 2);
  NVIC_EnableIRQ(TIM3_IRQn);
}

#ifdef __cplusplus
}
#endif
