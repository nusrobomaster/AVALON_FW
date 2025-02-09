################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/BRoCo/src/CANBus.cpp \
../System/BRoCo/src/CanSocketDriver.cpp \
../System/BRoCo/src/IOBus.cpp \
../System/BRoCo/src/MessageBus.cpp \
../System/BRoCo/src/NetworkBus.cpp \
../System/BRoCo/src/RoCanDriver.cpp \
../System/BRoCo/src/STMUARTDriver.cpp \
../System/BRoCo/src/UDevDriver.cpp 

OBJS += \
./System/BRoCo/src/CANBus.o \
./System/BRoCo/src/CanSocketDriver.o \
./System/BRoCo/src/IOBus.o \
./System/BRoCo/src/MessageBus.o \
./System/BRoCo/src/NetworkBus.o \
./System/BRoCo/src/RoCanDriver.o \
./System/BRoCo/src/STMUARTDriver.o \
./System/BRoCo/src/UDevDriver.o 

CPP_DEPS += \
./System/BRoCo/src/CANBus.d \
./System/BRoCo/src/CanSocketDriver.d \
./System/BRoCo/src/IOBus.d \
./System/BRoCo/src/MessageBus.d \
./System/BRoCo/src/NetworkBus.d \
./System/BRoCo/src/RoCanDriver.d \
./System/BRoCo/src/STMUARTDriver.d \
./System/BRoCo/src/UDevDriver.d 


# Each subdirectory must supply rules for building sources it contributes
System/BRoCo/src/%.o System/BRoCo/src/%.su System/BRoCo/src/%.cyclo: ../System/BRoCo/src/%.cpp System/BRoCo/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/stm_utils/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/Drivers/CMSIS/DSP/Include" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/comm/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/Threads/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/BRoCo" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/Build" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/Protocol" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-BRoCo-2f-src

clean-System-2f-BRoCo-2f-src:
	-$(RM) ./System/BRoCo/src/CANBus.cyclo ./System/BRoCo/src/CANBus.d ./System/BRoCo/src/CANBus.o ./System/BRoCo/src/CANBus.su ./System/BRoCo/src/CanSocketDriver.cyclo ./System/BRoCo/src/CanSocketDriver.d ./System/BRoCo/src/CanSocketDriver.o ./System/BRoCo/src/CanSocketDriver.su ./System/BRoCo/src/IOBus.cyclo ./System/BRoCo/src/IOBus.d ./System/BRoCo/src/IOBus.o ./System/BRoCo/src/IOBus.su ./System/BRoCo/src/MessageBus.cyclo ./System/BRoCo/src/MessageBus.d ./System/BRoCo/src/MessageBus.o ./System/BRoCo/src/MessageBus.su ./System/BRoCo/src/NetworkBus.cyclo ./System/BRoCo/src/NetworkBus.d ./System/BRoCo/src/NetworkBus.o ./System/BRoCo/src/NetworkBus.su ./System/BRoCo/src/RoCanDriver.cyclo ./System/BRoCo/src/RoCanDriver.d ./System/BRoCo/src/RoCanDriver.o ./System/BRoCo/src/RoCanDriver.su ./System/BRoCo/src/STMUARTDriver.cyclo ./System/BRoCo/src/STMUARTDriver.d ./System/BRoCo/src/STMUARTDriver.o ./System/BRoCo/src/STMUARTDriver.su ./System/BRoCo/src/UDevDriver.cyclo ./System/BRoCo/src/UDevDriver.d ./System/BRoCo/src/UDevDriver.o ./System/BRoCo/src/UDevDriver.su

.PHONY: clean-System-2f-BRoCo-2f-src

