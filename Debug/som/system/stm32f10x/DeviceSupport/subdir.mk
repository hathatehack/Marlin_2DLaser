################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../som/system/stm32f10x/DeviceSupport/stm32f10x_it.c \
../som/system/stm32f10x/DeviceSupport/system_stm32f10x.c 

S_UPPER_SRCS += \
../som/system/stm32f10x/DeviceSupport/startup_stm32f10x_md.S 

OBJS += \
./som/system/stm32f10x/DeviceSupport/startup_stm32f10x_md.o \
./som/system/stm32f10x/DeviceSupport/stm32f10x_it.o \
./som/system/stm32f10x/DeviceSupport/system_stm32f10x.o 

S_UPPER_DEPS += \
./som/system/stm32f10x/DeviceSupport/startup_stm32f10x_md.d 

C_DEPS += \
./som/system/stm32f10x/DeviceSupport/stm32f10x_it.d \
./som/system/stm32f10x/DeviceSupport/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
som/system/stm32f10x/DeviceSupport/%.o: ../som/system/stm32f10x/DeviceSupport/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler-with-cpp -I"../marlin" -I"../som/variants/ST103RE" -I"../som/cores/arduino" -I"../som/cores/arduino/avr" -I"../som/system/stm32f10x" -I"../som/system/stm32f10x/CoreSupport" -I"../som/system/stm32f10x/DeviceSupport" -I"../som/system/stm32f10x/StdPeriphDriver/inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

som/system/stm32f10x/DeviceSupport/%.o: ../som/system/stm32f10x/DeviceSupport/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -Dprintf=iprintf -DARDUINO=10708 -DF_CPU=72000000L -DARDUINO_STM32_F103VB -DARDUINO_ARCH_STM32F10X -DARDUINO_ARCH_STM32 -I"../marlin" -I"../som/variants/ST103RE" -I"../som/cores/arduino" -I"../som/cores/arduino/avr" -I"../som/system/stm32f10x" -I"../som/system/stm32f10x/CoreSupport" -I"../som/system/stm32f10x/DeviceSupport" -I"../som/system/stm32f10x/StdPeriphDriver/inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


