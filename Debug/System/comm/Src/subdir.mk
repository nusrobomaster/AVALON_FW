################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/comm/Src/System.cpp \
../System/comm/Src/Telemetry.cpp \
../System/comm/Src/Thread.cpp 

OBJS += \
./System/comm/Src/System.o \
./System/comm/Src/Telemetry.o \
./System/comm/Src/Thread.o 

CPP_DEPS += \
./System/comm/Src/System.d \
./System/comm/Src/Telemetry.d \
./System/comm/Src/Thread.d 


# Each subdirectory must supply rules for building sources it contributes
System/comm/Src/%.o System/comm/Src/%.su System/comm/Src/%.cyclo: ../System/comm/Src/%.cpp System/comm/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/stm_utils/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/Drivers/CMSIS/DSP/Include" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/comm" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/comm/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/Threads/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/BRoCo" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/Build" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/BRoCo/include/Protocol" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-comm-2f-Src

clean-System-2f-comm-2f-Src:
	-$(RM) ./System/comm/Src/System.cyclo ./System/comm/Src/System.d ./System/comm/Src/System.o ./System/comm/Src/System.su ./System/comm/Src/Telemetry.cyclo ./System/comm/Src/Telemetry.d ./System/comm/Src/Telemetry.o ./System/comm/Src/Telemetry.su ./System/comm/Src/Thread.cyclo ./System/comm/Src/Thread.d ./System/comm/Src/Thread.o ./System/comm/Src/Thread.su

.PHONY: clean-System-2f-comm-2f-Src

