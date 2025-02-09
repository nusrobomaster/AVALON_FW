################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Threads/Src/SuperCapCommThread.cpp \
../System/Threads/Src/dummy_thread.cpp 

OBJS += \
./System/Threads/Src/SuperCapCommThread.o \
./System/Threads/Src/dummy_thread.o 

CPP_DEPS += \
./System/Threads/Src/SuperCapCommThread.d \
./System/Threads/Src/dummy_thread.d 


# Each subdirectory must supply rules for building sources it contributes
System/Threads/Src/%.o System/Threads/Src/%.su System/Threads/Src/%.cyclo: ../System/Threads/Src/%.cpp System/Threads/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/stm_utils/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/Drivers/CMSIS/DSP/Include" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/comm/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/Threads/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/BRoCo" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/Build" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/Protocol" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Threads-2f-Src

clean-System-2f-Threads-2f-Src:
	-$(RM) ./System/Threads/Src/SuperCapCommThread.cyclo ./System/Threads/Src/SuperCapCommThread.d ./System/Threads/Src/SuperCapCommThread.o ./System/Threads/Src/SuperCapCommThread.su ./System/Threads/Src/dummy_thread.cyclo ./System/Threads/Src/dummy_thread.d ./System/Threads/Src/dummy_thread.o ./System/Threads/Src/dummy_thread.su

.PHONY: clean-System-2f-Threads-2f-Src

