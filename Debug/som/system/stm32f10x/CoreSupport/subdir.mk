################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../som/system/stm32f10x/CoreSupport/core_cm3.c 

OBJS += \
./som/system/stm32f10x/CoreSupport/core_cm3.o 

C_DEPS += \
./som/system/stm32f10x/CoreSupport/core_cm3.d 


# Each subdirectory must supply rules for building sources it contributes
som/system/stm32f10x/CoreSupport/%.o: ../som/system/stm32f10x/CoreSupport/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -Dprintf=iprintf -DARDUINO=10708 -DF_CPU=72000000L -DARDUINO_STM32_F103VB -DARDUINO_ARCH_STM32F10X -DARDUINO_ARCH_STM32 -I"../marlin" -I"../som/variants/ST103RE" -I"../som/cores/arduino" -I"../som/cores/arduino/avr" -I"../som/system/stm32f10x" -I"../som/system/stm32f10x/CoreSupport" -I"../som/system/stm32f10x/DeviceSupport" -I"../som/system/stm32f10x/StdPeriphDriver/inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


