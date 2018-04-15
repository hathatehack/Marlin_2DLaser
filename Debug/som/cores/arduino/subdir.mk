################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../som/cores/arduino/Wiring_digital.c \
../som/cores/arduino/cortex_handlers.c \
../som/cores/arduino/hooks.c \
../som/cores/arduino/itoa.c \
../som/cores/arduino/syscalls_stm32.c \
../som/cores/arduino/wiring.c \
../som/cores/arduino/wiring_analog.c \
../som/cores/arduino/wiring_shift.c 

CPP_SRCS += \
../som/cores/arduino/Print.cpp \
../som/cores/arduino/RingBuffer.cpp \
../som/cores/arduino/Stream.cpp \
../som/cores/arduino/USARTClass.cpp \
../som/cores/arduino/WMath.cpp \
../som/cores/arduino/WString.cpp \
../som/cores/arduino/cxxabi-compat.cpp \
../som/cores/arduino/main.cpp 

OBJS += \
./som/cores/arduino/Print.o \
./som/cores/arduino/RingBuffer.o \
./som/cores/arduino/Stream.o \
./som/cores/arduino/USARTClass.o \
./som/cores/arduino/WMath.o \
./som/cores/arduino/WString.o \
./som/cores/arduino/Wiring_digital.o \
./som/cores/arduino/cortex_handlers.o \
./som/cores/arduino/cxxabi-compat.o \
./som/cores/arduino/hooks.o \
./som/cores/arduino/itoa.o \
./som/cores/arduino/main.o \
./som/cores/arduino/syscalls_stm32.o \
./som/cores/arduino/wiring.o \
./som/cores/arduino/wiring_analog.o \
./som/cores/arduino/wiring_shift.o 

C_DEPS += \
./som/cores/arduino/Wiring_digital.d \
./som/cores/arduino/cortex_handlers.d \
./som/cores/arduino/hooks.d \
./som/cores/arduino/itoa.d \
./som/cores/arduino/syscalls_stm32.d \
./som/cores/arduino/wiring.d \
./som/cores/arduino/wiring_analog.d \
./som/cores/arduino/wiring_shift.d 

CPP_DEPS += \
./som/cores/arduino/Print.d \
./som/cores/arduino/RingBuffer.d \
./som/cores/arduino/Stream.d \
./som/cores/arduino/USARTClass.d \
./som/cores/arduino/WMath.d \
./som/cores/arduino/WString.d \
./som/cores/arduino/cxxabi-compat.d \
./som/cores/arduino/main.d 


# Each subdirectory must supply rules for building sources it contributes
som/cores/arduino/%.o: ../som/cores/arduino/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -Dprintf=iprintf -DARDUINO=10708 -DF_CPU=72000000L -DARDUINO_STM32_F103VB -DARDUINO_ARCH_STM32F10X -DARDUINO_ARCH_STM32 -I"../marlin" -I"../som/variants/ST103RE" -I"../som/cores/arduino" -I"../som/cores/arduino/avr" -I"../som/system/stm32f10x" -I"../som/system/stm32f10x/CoreSupport" -I"../som/system/stm32f10x/DeviceSupport" -I"../som/system/stm32f10x/StdPeriphDriver/inc" -fabi-version=0 -fno-exceptions -fno-rtti -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

som/cores/arduino/%.o: ../som/cores/arduino/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -Dprintf=iprintf -DARDUINO=10708 -DF_CPU=72000000L -DARDUINO_STM32_F103VB -DARDUINO_ARCH_STM32F10X -DARDUINO_ARCH_STM32 -I"../marlin" -I"../som/variants/ST103RE" -I"../som/cores/arduino" -I"../som/cores/arduino/avr" -I"../som/system/stm32f10x" -I"../som/system/stm32f10x/CoreSupport" -I"../som/system/stm32f10x/DeviceSupport" -I"../som/system/stm32f10x/StdPeriphDriver/inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


