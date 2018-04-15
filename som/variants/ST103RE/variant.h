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
#ifndef __VARIANT_SOM_H__
#define __VARIANT_SOM_H__

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC     8000000u
/* Master clock frequency */
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
#define VARIANT_MCK        72000000u
#elif defined(STM32F40_41xxx)
#define VARIANT_MCK       168000000u
#endif
 
/*-----------------------------------------------------------------
 *		Headers
 *-----------------------------------------------------------------*/
 
#include "Arduino.h"
#include "bitband.h"
#ifdef __cplusplus
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

#define NONE ((uint8_t)0xFF)

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

/* Number of pins defined in PinDescription array */
#define PINS_COUNT          (27u)
#define NUM_LED_PINS        (2u)
#define NUM_ANALOG_PINS     (4u)
#define NUM_DIGITAL_PINS    (PINS_COUNT - NUM_ANALOG_PINS)
#define A(p)                ((uint8_t)(NUM_DIGITAL_PINS + (p)))

/* Analog Pins */
extern const uint8_t A0;
extern const uint8_t A1;
extern const uint8_t A2;
extern const uint8_t A3;
#define ADC_RESOLUTION		(12u)

/* PWM */
#define PWM_FREQUENCY     (200000u)
#define PWM_RESOLUTION    (8u)
#define PWM_MAX_DUTYCYCLE (255u)
#define PWM_MIN_DUTYCYCLE (0u)

/* SPI Interfaces */
#define SPI_SS          (NUM_DIGITAL_PINS - NUM_LED_PINS - 4u)
#define SPI_SCK         (NUM_DIGITAL_PINS - NUM_LED_PINS - 3u)
#define SPI_MISO        (NUM_DIGITAL_PINS - NUM_LED_PINS - 2u)
#define SPI_MOSI        (NUM_DIGITAL_PINS - NUM_LED_PINS - 1u)

/* Wire Interfaces */


/* USART Interfaces */
// Serial0
#define TX              (UART - 4u)
#define RX              (UART - 3u)
// Serial1
#define TX1             (UART - 2u)
#define RX1             (UART - 1u)
#define UART           (NUM_DIGITAL_PINS - NUM_LED_PINS - 4u)

/* USB Interfaces */


/* LEDs */
#define PIN_LED_RUN         (NUM_DIGITAL_PINS - 2u)
#define PIN_LED_STATUS      (NUM_DIGITAL_PINS - 1u)
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
#define LED_RUN             PAout(5)//(*(__IO unsigned long*)(0x422201A8))
#define LED_STATUS          PCout(4)//(*(__IO unsigned long*)(0x422201AC))
#elif defined(STM32F40_41xxx)
#define LED_RUN             (*(__IO unsigned long*)(0x424102A8))
#define LED_STATUS          (*(__IO unsigned long*)(0x424102AC))
#endif


extern uint16_t TIM_ARR;
extern TIM_TimeBaseInitTypeDef gTIM_TimeBaseStructure;
extern TIM_OCInitTypeDef       gTIM_OCInitStructure;

#ifdef __cplusplus
}
#endif

/*-----------------------------------------------------------
 *		Arduino objects - C++ only
 *-----------------------------------------------------------*/
#ifdef __cplusplus
extern USARTClass Serial;
extern USARTClass Serial1;
#endif

#endif  /* __VARIANT_SOM_H__ */
