################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/stm_utils/Src/adc_utils.cpp \
../System/stm_utils/Src/msg_handler.cpp \
../System/stm_utils/Src/supercap_controllers.cpp 

OBJS += \
./System/stm_utils/Src/adc_utils.o \
./System/stm_utils/Src/msg_handler.o \
./System/stm_utils/Src/supercap_controllers.o 

CPP_DEPS += \
./System/stm_utils/Src/adc_utils.d \
./System/stm_utils/Src/msg_handler.d \
./System/stm_utils/Src/supercap_controllers.d 


# Each subdirectory must supply rules for building sources it contributes
System/stm_utils/Src/%.o System/stm_utils/Src/%.su System/stm_utils/Src/%.cyclo: ../System/stm_utils/Src/%.cpp System/stm_utils/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ngwei/OneDrive/Documents/AVALON_FW/System/stm_utils/Inc" -I"C:/Users/ngwei/OneDrive/Documents/AVALON_FW/Drivers/CMSIS/DSP/Include" -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-stm_utils-2f-Src

clean-System-2f-stm_utils-2f-Src:
	-$(RM) ./System/stm_utils/Src/adc_utils.cyclo ./System/stm_utils/Src/adc_utils.d ./System/stm_utils/Src/adc_utils.o ./System/stm_utils/Src/adc_utils.su ./System/stm_utils/Src/msg_handler.cyclo ./System/stm_utils/Src/msg_handler.d ./System/stm_utils/Src/msg_handler.o ./System/stm_utils/Src/msg_handler.su ./System/stm_utils/Src/supercap_controllers.cyclo ./System/stm_utils/Src/supercap_controllers.d ./System/stm_utils/Src/supercap_controllers.o ./System/stm_utils/Src/supercap_controllers.su

.PHONY: clean-System-2f-stm_utils-2f-Src

