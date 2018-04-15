/*
  Copyright (c) 2016 hathatehack  All right reserved.
  Copyright (c) 2012 Arduino.  All right reserved.

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

static void __halt() {
	// Halts
	while (1)
		;
}

extern void svcHook(void);
extern void pendSVHook(void);
extern int sysTickHook(void);

/* Cortex-M3/M4 core handlers */
void NMI_Handler       (void) __attribute__ ((weak, alias("__halt")));
void HardFault_Handler (void) __attribute__ ((weak, alias("__halt")));
void MemManage_Handler (void) __attribute__ ((weak, alias("__halt")));
void BusFault_Handler  (void) __attribute__ ((weak, alias("__halt")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("__halt")));
void DebugMon_Handler  (void) __attribute__ ((weak, alias("__halt")));
void SVC_Handler       (void) { svcHook(); }
void PendSV_Handler    (void) {	pendSVHook(); }
void SysTick_Handler(void)
{
	// Increment tick count each ms
	TimeTick_Increment();

	extern volatile uint32_t TimingMillis, TimingMillis_led;
//	if(TimingMillis - TimingMillis_led >= ledruntime) {
//	  TimingMillis_led = TimingMillis;
//	  LED_RUN = !LED_RUN;
//	}
}

/* Peripherals handlers */
#define DCLpHandler(pHandler) void (pHandler)(void) __attribute__ ((weak, alias("__halt")))
typedef void (*const pHandler)(void);

#if defined(STM32F10X_LD) || defined(STM32F10X_LD_VL) || defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) || defined(STM32F10X_XL) || defined(STM32F10X_CL)
DCLpHandler(WWDG_IRQHandler);
DCLpHandler(PVD_IRQHandler);
DCLpHandler(TAMPER_IRQHandler);
DCLpHandler(RTC_IRQHandler);
DCLpHandler(FLASH_IRQHandler);
DCLpHandler(RCC_IRQHandler);
DCLpHandler(EXTI0_IRQHandler);
DCLpHandler(EXTI1_IRQHandler);
DCLpHandler(EXTI2_IRQHandler);
DCLpHandler(EXTI3_IRQHandler);
DCLpHandler(EXTI4_IRQHandler);
DCLpHandler(DMA1_Channel1_IRQHandler);
DCLpHandler(DMA1_Channel2_IRQHandler);
DCLpHandler(DMA1_Channel3_IRQHandler);
DCLpHandler(DMA1_Channel4_IRQHandler);
DCLpHandler(DMA1_Channel5_IRQHandler);
DCLpHandler(DMA1_Channel6_IRQHandler);
DCLpHandler(DMA1_Channel7_IRQHandler);
DCLpHandler(ADC1_2_IRQHandler);
DCLpHandler(USB_HP_CAN1_TX_IRQHandler);
DCLpHandler(USB_LP_CAN1_RX0_IRQHandler);
DCLpHandler(CAN1_RX1_IRQHandler);
DCLpHandler(CAN1_SCE_IRQHandler);
DCLpHandler(EXTI9_5_IRQHandler);
DCLpHandler(TIM1_BRK_TIM9_IRQHandler);
DCLpHandler(TIM1_UP_TIM10_IRQHandler);
DCLpHandler(TIM1_TRG_COM_TIM11_IRQHandler);
DCLpHandler(TIM1_BRK_IRQHandler);
DCLpHandler(TIM1_UP_IRQHandler);
DCLpHandler(TIM1_TRG_COM_IRQHandler);
DCLpHandler(TIM1_CC_IRQHandler);
DCLpHandler(TIM2_IRQHandler);
DCLpHandler(TIM3_IRQHandler);
DCLpHandler(TIM4_IRQHandler);
DCLpHandler(I2C1_EV_IRQHandler);
DCLpHandler(I2C1_ER_IRQHandler);
DCLpHandler(I2C2_EV_IRQHandler);
DCLpHandler(I2C2_ER_IRQHandler);
DCLpHandler(SPI1_IRQHandler);
DCLpHandler(SPI2_IRQHandler);
DCLpHandler(USART1_IRQHandler);
DCLpHandler(USART2_IRQHandler);
DCLpHandler(USART3_IRQHandler);
DCLpHandler(EXTI15_10_IRQHandler);
DCLpHandler(RTCAlarm_IRQHandler);
DCLpHandler(USBWakeUp_IRQHandler);
DCLpHandler(TIM8_BRK_IRQHandler);
DCLpHandler(TIM8_UP_IRQHandler);
DCLpHandler(TIM8_TRG_COM_IRQHandler);
DCLpHandler(TIM8_BRK_TIM12_IRQHandler);
DCLpHandler(TIM8_UP_TIM13_IRQHandler);
DCLpHandler(TIM8_TRG_COM_TIM14_IRQHandler);
DCLpHandler(TIM8_CC_IRQHandler);
DCLpHandler(ADC3_IRQHandler);
DCLpHandler(FSMC_IRQHandler);
DCLpHandler(SDIO_IRQHandler);
DCLpHandler(TIM5_IRQHandler);
DCLpHandler(SPI3_IRQHandler);
DCLpHandler(UART4_IRQHandler);
DCLpHandler(UART5_IRQHandler);
DCLpHandler(TIM6_IRQHandler);
DCLpHandler(TIM7_IRQHandler);
DCLpHandler(DMA2_Channel1_IRQHandler);
DCLpHandler(DMA2_Channel2_IRQHandler);
DCLpHandler(DMA2_Channel3_IRQHandler);
DCLpHandler(DMA2_Channel4_5_IRQHandler);
DCLpHandler(CAN1_TX_IRQHandler);
DCLpHandler(CAN1_RX0_IRQHandler);
DCLpHandler(OTG_FS_WKUP_IRQHandler);
DCLpHandler(DMA2_Channel4_IRQHandler);
DCLpHandler(DMA2_Channel5_IRQHandler);
DCLpHandler(ETH_IRQHandler);
DCLpHandler(ETH_WKUP_IRQHandler);
DCLpHandler(CAN2_TX_IRQHandler);
DCLpHandler(CAN2_RX0_IRQHandler);
DCLpHandler(CAN2_RX1_IRQHandler);
DCLpHandler(CAN2_SCE_IRQHandler);
DCLpHandler(OTG_FS_IRQHandler);
DCLpHandler(TIM1_BRK_TIM15_IRQHandler);
DCLpHandler(TIM1_UP_TIM16_IRQHandler);
DCLpHandler(TIM1_TRG_COM_TIM17_IRQHandler);
DCLpHandler(CEC_IRQHandler);
DCLpHandler(TIM6_DAC_IRQHandler);
DCLpHandler(TIM12_IRQHandler);
DCLpHandler(TIM13_IRQHandler);
DCLpHandler(TIM14_IRQHandler);
#elif defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F401xx) || defined(STM32F411xE) || defined(STM32F446xx)
DCLpHandler(WWDG_IRQHandler);
DCLpHandler(PVD_IRQHandler);
DCLpHandler(TAMP_STAMP_IRQHandler);
DCLpHandler(RTC_WKUP_IRQHandler);
DCLpHandler(FLASH_IRQHandler);
DCLpHandler(RCC_IRQHandler);
DCLpHandler(EXTI0_IRQHandler);
DCLpHandler(EXTI1_IRQHandler);
DCLpHandler(EXTI2_IRQHandler);
DCLpHandler(EXTI3_IRQHandler);
DCLpHandler(EXTI4_IRQHandler);
DCLpHandler(DMA1_Stream0_IRQHandler);
DCLpHandler(DMA1_Stream1_IRQHandler);
DCLpHandler(DMA1_Stream2_IRQHandler);
DCLpHandler(DMA1_Stream3_IRQHandler);
DCLpHandler(DMA1_Stream4_IRQHandler);
DCLpHandler(DMA1_Stream5_IRQHandler);
DCLpHandler(DMA1_Stream6_IRQHandler);
DCLpHandler(ADC_IRQHandler);
DCLpHandler(CAN1_TX_IRQHandler);
DCLpHandler(CAN1_RX0_IRQHandler);
DCLpHandler(CAN1_RX1_IRQHandler);
DCLpHandler(CAN1_SCE_IRQHandler);
DCLpHandler(EXTI9_5_IRQHandler);
DCLpHandler(TIM1_BRK_TIM9_IRQHandler);
DCLpHandler(TIM1_UP_TIM10_IRQHandler);
DCLpHandler(TIM1_TRG_COM_TIM11_IRQHandler);
DCLpHandler(TIM1_CC_IRQHandler);
DCLpHandler(TIM2_IRQHandler);
DCLpHandler(TIM3_IRQHandler);
DCLpHandler(TIM4_IRQHandler);
DCLpHandler(I2C1_EV_IRQHandler);
DCLpHandler(I2C1_ER_IRQHandler);
DCLpHandler(I2C2_EV_IRQHandler);
DCLpHandler(I2C2_ER_IRQHandler);
DCLpHandler(SPI1_IRQHandler);
DCLpHandler(SPI2_IRQHandler);
DCLpHandler(USART1_IRQHandler);
DCLpHandler(USART2_IRQHandler);
DCLpHandler(USART3_IRQHandler);
DCLpHandler(EXTI15_10_IRQHandler);
DCLpHandler(RTC_Alarm_IRQHandler);
DCLpHandler(OTG_FS_WKUP_IRQHandler);
DCLpHandler(TIM8_BRK_TIM12_IRQHandler);
DCLpHandler(TIM8_UP_TIM13_IRQHandler);
DCLpHandler(TIM8_TRG_COM_TIM14_IRQHandler);
DCLpHandler(TIM8_CC_IRQHandler);
DCLpHandler(DMA1_Stream7_IRQHandler);
DCLpHandler(FMC_IRQHandler);
DCLpHandler(SDIO_IRQHandler);
DCLpHandler(TIM5_IRQHandler);
DCLpHandler(SPI3_IRQHandler);
DCLpHandler(UART4_IRQHandler);
DCLpHandler(UART5_IRQHandler);
DCLpHandler(TIM6_DAC_IRQHandler);
DCLpHandler(TIM7_IRQHandler);
DCLpHandler(DMA2_Stream0_IRQHandler);
DCLpHandler(DMA2_Stream1_IRQHandler);
DCLpHandler(DMA2_Stream2_IRQHandler);
DCLpHandler(DMA2_Stream3_IRQHandler);
DCLpHandler(DMA2_Stream4_IRQHandler);
DCLpHandler(ETH_IRQHandler);
DCLpHandler(ETH_WKUP_IRQHandler);
DCLpHandler(CAN2_TX_IRQHandler);
DCLpHandler(CAN2_RX0_IRQHandler);
DCLpHandler(CAN2_RX1_IRQHandler);
DCLpHandler(CAN2_SCE_IRQHandler);
DCLpHandler(OTG_FS_IRQHandler);
DCLpHandler(DMA2_Stream5_IRQHandler);
DCLpHandler(DMA2_Stream6_IRQHandler);
DCLpHandler(DMA2_Stream7_IRQHandler);
DCLpHandler(USART6_IRQHandler);
DCLpHandler(I2C3_EV_IRQHandler);
DCLpHandler(I2C3_ER_IRQHandler);
DCLpHandler(OTG_HS_EP1_OUT_IRQHandler);
DCLpHandler(OTG_HS_EP1_IN_IRQHandler);
DCLpHandler(OTG_HS_WKUP_IRQHandler);
DCLpHandler(OTG_HS_IRQHandler);
DCLpHandler(DCMI_IRQHandler);
DCLpHandler(CRYP_IRQHandler);
DCLpHandler(HASH_RNG_IRQHandler);
DCLpHandler(FPU_IRQHandler);
DCLpHandler(UART7_IRQHandler);
DCLpHandler(UART8_IRQHandler);
DCLpHandler(SPI4_IRQHandler);
DCLpHandler(SPI5_IRQHandler);
DCLpHandler(SPI6_IRQHandler);
DCLpHandler(SAI1_IRQHandler);
DCLpHandler(LTDC_IRQHandler);
DCLpHandler(LTDC_ER_IRQHandler);
DCLpHandler(DMA2D_IRQHandler);
DCLpHandler(SAI2_IRQHandler);
DCLpHandler(QUADSPI_IRQHandler);
DCLpHandler(CEC_IRQHandler);
DCLpHandler(SPDIF_RX_IRQHandler);
DCLpHandler(FMPI2C1_EV_IRQHandler);
DCLpHandler(FMPI2C1_ER_IRQHandler);
#endif

__attribute__ ((section(".isr_vectors"),used))
const pHandler __isr_vectors[] = {
	//Chip Level - STM32F10x
#if defined(STM32F10X_LD) || defined(STM32F10X_LD_VL) || defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) || defined(STM32F10X_XL) || defined(STM32F10X_CL)
	WWDG_IRQHandler                 ,// Window Watchdog
	PVD_IRQHandler                  ,// PVD through EXTI Line detect
	TAMPER_IRQHandler               ,// Tamper
	RTC_IRQHandler                  ,// RTC
	FLASH_IRQHandler                ,// Flash
	RCC_IRQHandler                  ,// RCC
	EXTI0_IRQHandler                ,// EXTI Line 0
	EXTI1_IRQHandler                ,// EXTI Line 1
	EXTI2_IRQHandler                ,// EXTI Line 2
	EXTI3_IRQHandler                ,// EXTI Line 3
	EXTI4_IRQHandler                ,// EXTI Line 4
	DMA1_Channel1_IRQHandler        ,// DMA1 Channel 1
	DMA1_Channel2_IRQHandler        ,// DMA1 Channel 2
	DMA1_Channel3_IRQHandler        ,// DMA1 Channel 3
	DMA1_Channel4_IRQHandler        ,// DMA1 Channel 4
	DMA1_Channel5_IRQHandler        ,// DMA1 Channel 5
	DMA1_Channel6_IRQHandler        ,// DMA1 Channel 6
	DMA1_Channel7_IRQHandler        ,// DMA1 Channel 7
#elif defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F401xx) || defined(STM32F411xE) || defined(STM32F446xx)
    WWDG_IRQHandler,                   // Window WatchDog
    PVD_IRQHandler,                    // PVD through EXTI Line detection
    TAMP_STAMP_IRQHandler,             // Tamper and TimeStamps through the EXTI line
    RTC_WKUP_IRQHandler,               // RTC Wakeup through the EXTI line
    FLASH_IRQHandler,                  // FLASH
    RCC_IRQHandler,                    // RCC
    EXTI0_IRQHandler,                  // EXTI Line0
    EXTI1_IRQHandler,                  // EXTI Line1
    EXTI2_IRQHandler,                  // EXTI Line2
    EXTI3_IRQHandler,                  // EXTI Line3
    EXTI4_IRQHandler,                  // EXTI Line4
    DMA1_Stream0_IRQHandler,           // DMA1 Stream 0
    DMA1_Stream1_IRQHandler,           // DMA1 Stream 1
    DMA1_Stream2_IRQHandler,           // DMA1 Stream 2
    DMA1_Stream3_IRQHandler,           // DMA1 Stream 3
    DMA1_Stream4_IRQHandler,           // DMA1 Stream 4
    DMA1_Stream5_IRQHandler,           // DMA1 Stream 5
    DMA1_Stream6_IRQHandler,           // DMA1 Stream 6
    ADC_IRQHandler,                    // ADC1, ADC2 and ADC3s
#endif
#if defined(STM32F10X_LD)
	  ADC1_2_IRQHandler             ,// ADC1_2
	  USB_HP_CAN1_TX_IRQHandler     ,// USB High Priority or CAN1 TX
	  USB_LP_CAN1_RX0_IRQHandler    ,// USB Low  Priority or CAN1 RX0
	  CAN1_RX1_IRQHandler           ,// CAN1 RX1
	  CAN1_SCE_IRQHandler           ,// CAN1 SCE
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_IRQHandler           ,// TIM1 Break
	  TIM1_UP_IRQHandler            ,// TIM1 Update
	  TIM1_TRG_COM_IRQHandler       ,// TIM1 Trigger and Commutation
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  0                             ,// Reserved
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  0                             ,// Reserved
	  0                             ,// Reserved
	  SPI1_IRQHandler               ,// SPI1
	  0                             ,// Reserved
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  0                             ,// Reserved
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  USBWakeUp_IRQHandler          ,// USB Wakeup from suspend
#elif defined(STM32F10X_LD_VL)
	  ADC1_IRQHandler               ,// ADC1
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_TIM15_IRQHandler     ,// TIM1 Break and TIM15
	  TIM1_UP_TIM16_IRQHandler      ,// TIM1 Update and TIM16
	  TIM1_TRG_COM_TIM17_IRQHandler ,// TIM1 Trigger and Commutation and TIM17
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  0                             ,// Reserved
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  0                             ,// Reserved
	  0                             ,// Reserved
	  SPI1_IRQHandler               ,// SPI1
	  0                             ,// Reserved
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  0                             ,// Reserved
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  CEC_IRQHandler                ,// HDMI-CEC
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  TIM6_DAC_IRQHandler           ,// TIM6 and DAC underrun
	  TIM7_IRQHandler               ,// TIM7
#elif defined(STM32F10X_MD)
	  ADC1_2_IRQHandler             ,// ADC1_2
	  USB_HP_CAN1_TX_IRQHandler     ,// USB High Priority or CAN1 TX
	  USB_LP_CAN1_RX0_IRQHandler    ,// USB Low  Priority or CAN1 RX0
	  CAN1_RX1_IRQHandler           ,// CAN1 RX1
	  CAN1_SCE_IRQHandler           ,// CAN1 SCE
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_IRQHandler           ,// TIM1 Break
	  TIM1_UP_IRQHandler            ,// TIM1 Update
	  TIM1_TRG_COM_IRQHandler       ,// TIM1 Trigger and Commutation
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  TIM4_IRQHandler               ,// TIM4
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  I2C2_EV_IRQHandler            ,// I2C2 Event
	  I2C2_ER_IRQHandler            ,// I2C2 Error
	  SPI1_IRQHandler               ,// SPI1
	  SPI2_IRQHandler               ,// SPI2
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  USART3_IRQHandler             ,// USART3
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  USBWakeUp_IRQHandler          ,// USB Wakeup from suspend
#elif defined(STM32F10X_MD_VL)
	  ADC1_IRQHandler               ,// ADC1
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_TIM15_IRQHandler     ,// TIM1 Break and TIM15
	  TIM1_UP_TIM16_IRQHandler      ,// TIM1 Update and TIM16
	  TIM1_TRG_COM_TIM17_IRQHandler ,// TIM1 Trigger and Commutation and TIM17
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  TIM4_IRQHandler               ,// TIM4
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  I2C2_EV_IRQHandler            ,// I2C2 Event
	  I2C2_ER_IRQHandler            ,// I2C2 Error
	  SPI1_IRQHandler               ,// SPI1
	  SPI2_IRQHandler               ,// SPI2
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  USART3_IRQHandler             ,// USART3
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  CEC_IRQHandler                ,// HDMI-CEC
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  TIM6_DAC_IRQHandler           ,// TIM6 and DAC underrun
	  TIM7_IRQHandler               ,// TIM7
#elif defined(STM32F10X_HD)
	  ADC1_2_IRQHandler             ,// ADC1 & ADC2
	  USB_HP_CAN1_TX_IRQHandler     ,// USB High Priority or CAN1 TX
	  USB_LP_CAN1_RX0_IRQHandler    ,// USB Low  Priority or CAN1 RX0
	  CAN1_RX1_IRQHandler           ,// CAN1 RX1
	  CAN1_SCE_IRQHandler           ,// CAN1 SCE
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_IRQHandler           ,// TIM1 Break
	  TIM1_UP_IRQHandler            ,// TIM1 Update
	  TIM1_TRG_COM_IRQHandler       ,// TIM1 Trigger and Commutation
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  TIM4_IRQHandler               ,// TIM4
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  I2C2_EV_IRQHandler            ,// I2C2 Event
	  I2C2_ER_IRQHandler            ,// I2C2 Error
	  SPI1_IRQHandler               ,// SPI1
	  SPI2_IRQHandler               ,// SPI2
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  USART3_IRQHandler             ,// USART3
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  USBWakeUp_IRQHandler          ,// USB Wakeup from suspend
	  TIM8_BRK_IRQHandler           ,// TIM8 Break
	  TIM8_UP_IRQHandler            ,// TIM8 Update
	  TIM8_TRG_COM_IRQHandler       ,// TIM8 Trigger and Commutation
	  TIM8_CC_IRQHandler            ,// TIM8 Capture Compare
	  ADC3_IRQHandler               ,// ADC3
	  FSMC_IRQHandler               ,// FSMC
	  SDIO_IRQHandler               ,// SDIO
	  TIM5_IRQHandler               ,// TIM5
	  SPI3_IRQHandler               ,// SPI3
	  UART4_IRQHandler              ,// UART4
	  UART5_IRQHandler              ,// UART5
	  TIM6_IRQHandler               ,// TIM6
	  TIM7_IRQHandler               ,// TIM7
	  DMA2_Channel1_IRQHandler      ,// DMA2 Channel1
	  DMA2_Channel2_IRQHandler      ,// DMA2 Channel2
	  DMA2_Channel3_IRQHandler      ,// DMA2 Channel3
	  DMA2_Channel4_5_IRQHandler    ,// DMA2 Channel4 & Channel5
#elif defined(STM32F10X_HD_VL)
	  ADC1_IRQHandler               ,// ADC1
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_TIM15_IRQHandler     ,// TIM1 Break and TIM15
	  TIM1_UP_TIM16_IRQHandler      ,// TIM1 Update and TIM16
	  TIM1_TRG_COM_TIM17_IRQHandler ,// TIM1 Trigger and Commutation and TIM17
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  TIM4_IRQHandler               ,// TIM4
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  I2C2_EV_IRQHandler            ,// I2C2 Event
	  I2C2_ER_IRQHandler            ,// I2C2 Error
	  SPI1_IRQHandler               ,// SPI1
	  SPI2_IRQHandler               ,// SPI2
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  USART3_IRQHandler             ,// USART3
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  CEC_IRQHandler                ,// HDMI-CEC
	  TIM12_IRQHandler              ,// TIM12
	  TIM13_IRQHandler              ,// TIM13
	  TIM14_IRQHandler              ,// TIM14
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  TIM5_IRQHandler               ,// TIM5
	  SPI3_IRQHandler               ,// SPI3
	  UART4_IRQHandler              ,// UART4
	  UART5_IRQHandler              ,// UART5
	  TIM6_DAC_IRQHandler           ,// TIM6 and DAC underrun
	  TIM7_IRQHandler               ,// TIM7
	  DMA2_Channel1_IRQHandler      ,// DMA2 Channel1
	  DMA2_Channel2_IRQHandler      ,// DMA2 Channel2
	  DMA2_Channel3_IRQHandler      ,// DMA2 Channel3
	  DMA2_Channel4_5_IRQHandler    ,// DMA2 Channel4 & Channel5
	  DMA2_Channel5_IRQHandler      ,// DMA2 Channel5
#elif defined(STM32F10X_XL)
	  ADC1_2_IRQHandler             ,// ADC1 & ADC2
	  USB_HP_CAN1_TX_IRQHandler     ,// USB High Priority or CAN1 TX
	  USB_LP_CAN1_RX0_IRQHandler    ,// USB Low  Priority or CAN1 RX0
	  CAN1_RX1_IRQHandler           ,// CAN1 RX1
	  CAN1_SCE_IRQHandler           ,// CAN1 SCE
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_TIM9_IRQHandler      ,// TIM1 Break and TIM9
	  TIM1_UP_TIM10_IRQHandler      ,// TIM1 Update and TIM10
	  TIM1_TRG_COM_TIM11_IRQHandler ,// TIM1 Trigger and Commutation and TIM11
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  TIM4_IRQHandler               ,// TIM4
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  I2C2_EV_IRQHandler            ,// I2C2 Event
	  I2C2_ER_IRQHandler            ,// I2C2 Error
	  SPI1_IRQHandler               ,// SPI1
	  SPI2_IRQHandler               ,// SPI2
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  USART3_IRQHandler             ,// USART3
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC Alarm through EXTI Line
	  USBWakeUp_IRQHandler          ,// USB Wakeup from suspend
	  TIM8_BRK_TIM12_IRQHandler     ,// TIM8 Break and TIM12
	  TIM8_UP_TIM13_IRQHandler      ,// TIM8 Update and TIM13
	  TIM8_TRG_COM_TIM14_IRQHandler ,// TIM8 Trigger and Commutation and TIM14
	  TIM8_CC_IRQHandler            ,// TIM8 Capture Compare
	  ADC3_IRQHandler               ,// ADC3
	  FSMC_IRQHandler               ,// FSMC
	  SDIO_IRQHandler               ,// SDIO
	  TIM5_IRQHandler               ,// TIM5
	  SPI3_IRQHandler               ,// SPI3
	  UART4_IRQHandler              ,// UART4
	  UART5_IRQHandler              ,// UART5
	  TIM6_IRQHandler               ,// TIM6
	  TIM7_IRQHandler               ,// TIM7
	  DMA2_Channel1_IRQHandler      ,// DMA2 Channel1
	  DMA2_Channel2_IRQHandler      ,// DMA2 Channel2
	  DMA2_Channel3_IRQHandler      ,// DMA2 Channel3
	  DMA2_Channel4_5_IRQHandler    ,// DMA2 Channel4 & Channel5
#elif defined(STM32F10X_CL)
	  ADC1_2_IRQHandler             ,// ADC1 and ADC2
	  CAN1_TX_IRQHandler            ,// CAN1 TX
	  CAN1_RX0_IRQHandler           ,// CAN1 RX0
	  CAN1_RX1_IRQHandler           ,// CAN1 RX1
	  CAN1_SCE_IRQHandler           ,// CAN1 SCE
	  EXTI9_5_IRQHandler            ,// EXTI Line 9..5
	  TIM1_BRK_IRQHandler           ,// TIM1 Break
	  TIM1_UP_IRQHandler            ,// TIM1 Update
	  TIM1_TRG_COM_IRQHandler       ,// TIM1 Trigger and Commutation
	  TIM1_CC_IRQHandler            ,// TIM1 Capture Compare
	  TIM2_IRQHandler               ,// TIM2
	  TIM3_IRQHandler               ,// TIM3
	  TIM4_IRQHandler               ,// TIM4
	  I2C1_EV_IRQHandler            ,// I2C1 Event
	  I2C1_ER_IRQHandler            ,// I2C1 Error
	  I2C2_EV_IRQHandler            ,// I2C2 Event
	  I2C2_ER_IRQHandler            ,// I2C1 Error
	  SPI1_IRQHandler               ,// SPI1
	  SPI2_IRQHandler               ,// SPI2
	  USART1_IRQHandler             ,// USART1
	  USART2_IRQHandler             ,// USART2
	  USART3_IRQHandler             ,// USART3
	  EXTI15_10_IRQHandler          ,// EXTI Line 15..10
	  RTCAlarm_IRQHandler           ,// RTC alarm through EXTI line
	  OTG_FS_WKUP_IRQHandler        ,// USB OTG FS Wakeup through EXTI line
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  0                             ,// Reserved
	  TIM5_IRQHandler               ,// TIM5
	  SPI3_IRQHandler               ,// SPI3
	  UART4_IRQHandler              ,// UART4
	  UART5_IRQHandler              ,// UART5
	  TIM6_IRQHandler               ,// TIM6
	  TIM7_IRQHandler               ,// TIM7
	  DMA2_Channel1_IRQHandler      ,// DMA2 Channel1
	  DMA2_Channel2_IRQHandler      ,// DMA2 Channel2
	  DMA2_Channel3_IRQHandler      ,// DMA2 Channel3
	  DMA2_Channel4_IRQHandler      ,// DMA2 Channel4
	  DMA2_Channel5_IRQHandler      ,// DMA2 Channel5
	  ETH_IRQHandler                ,// Ethernet
	  ETH_WKUP_IRQHandler           ,// Ethernet Wakeup through EXTI line
	  CAN2_TX_IRQHandler            ,// CAN2 TX
	  CAN2_RX0_IRQHandler           ,// CAN2 RX0
	  CAN2_RX1_IRQHandler           ,// CAN2 RX1
	  CAN2_SCE_IRQHandler           ,// CAN2 SCE
	  OTG_FS_IRQHandler             ,// USB OTG FS
#elif defined(STM32F40_41xxx)
    CAN1_TX_IRQHandler,                // CAN1 TX
    CAN1_RX0_IRQHandler,               // CAN1 RX0
    CAN1_RX1_IRQHandler,               // CAN1 RX1
    CAN1_SCE_IRQHandler,               // CAN1 SCE
    EXTI9_5_IRQHandler,                // External Line[9:5]s
    TIM1_BRK_TIM9_IRQHandler,          // TIM1 Break and TIM9
    TIM1_UP_TIM10_IRQHandler,          // TIM1 Update and TIM10
    TIM1_TRG_COM_TIM11_IRQHandler,     // TIM1 Trigger and Commutation and TIM11
    TIM1_CC_IRQHandler,                // TIM1 Capture Compare
    TIM2_IRQHandler,                   // TIM2
    TIM3_IRQHandler,                   // TIM3
    TIM4_IRQHandler,                   // TIM4
    I2C1_EV_IRQHandler,                // I2C1 Event
    I2C1_ER_IRQHandler,                // I2C1 Error
    I2C2_EV_IRQHandler,                // I2C2 Event
    I2C2_ER_IRQHandler,                // I2C2 Error
    SPI1_IRQHandler,                   // SPI1
    SPI2_IRQHandler,                   // SPI2
    USART1_IRQHandler,                 // USART1
    USART2_IRQHandler,                 // USART2
    USART3_IRQHandler,                 // USART3
    EXTI15_10_IRQHandler,              // External Line[15:10]s
    RTC_Alarm_IRQHandler,              // RTC Alarm (A and B) through EXTI Line
    OTG_FS_WKUP_IRQHandler,            // USB OTG FS Wakeup through EXTI line
    TIM8_BRK_TIM12_IRQHandler,         // TIM8 Break and TIM12
    TIM8_UP_TIM13_IRQHandler,          // TIM8 Update and TIM13
    TIM8_TRG_COM_TIM14_IRQHandler,     // TIM8 Trigger and Commutation and TIM14
    TIM8_CC_IRQHandler,                // TIM8 Capture Compare
    DMA1_Stream7_IRQHandler,           // DMA1 Stream7
    FMC_IRQHandler,                    // FMC
    SDIO_IRQHandler,                   // SDIO
    TIM5_IRQHandler,                   // TIM5
    SPI3_IRQHandler,                   // SPI3
    UART4_IRQHandler,                  // UART4
    UART5_IRQHandler,                  // UART5
    TIM6_DAC_IRQHandler,               // TIM6 and DAC1&2 underrun errors
    TIM7_IRQHandler,                   // TIM7
    DMA2_Stream0_IRQHandler,           // DMA2 Stream 0
    DMA2_Stream1_IRQHandler,           // DMA2 Stream 1
    DMA2_Stream2_IRQHandler,           // DMA2 Stream 2
    DMA2_Stream3_IRQHandler,           // DMA2 Stream 3
    DMA2_Stream4_IRQHandler,           // DMA2 Stream 4
    ETH_IRQHandler,                    // Ethernet
    ETH_WKUP_IRQHandler,               // Ethernet Wakeup through EXTI line
    CAN2_TX_IRQHandler,                // CAN2 TX
    CAN2_RX0_IRQHandler,               // CAN2 RX0
    CAN2_RX1_IRQHandler,               // CAN2 RX1
    CAN2_SCE_IRQHandler,               // CAN2 SCE
    OTG_FS_IRQHandler,                 // USB OTG FS
    DMA2_Stream5_IRQHandler,           // DMA2 Stream 5
    DMA2_Stream6_IRQHandler,           // DMA2 Stream 6
    DMA2_Stream7_IRQHandler,           // DMA2 Stream 7
    USART6_IRQHandler,                 // USART6
    I2C3_EV_IRQHandler,                // I2C3 event
    I2C3_ER_IRQHandler,                // I2C3 error
    OTG_HS_EP1_OUT_IRQHandler,         // USB OTG HS End Point 1 Out
    OTG_HS_EP1_IN_IRQHandler,          // USB OTG HS End Point 1 In
    OTG_HS_WKUP_IRQHandler,            // USB OTG HS Wakeup through EXTI
    OTG_HS_IRQHandler,                 // USB OTG HS
    DCMI_IRQHandler,                   // DCMI
    CRYP_IRQHandler,                   // CRYP
    HASH_RNG_IRQHandler,               // Hash and Rng
    FPU_IRQHandler,                    // FPU
#endif
};

#ifdef __cplusplus
}
#endif
