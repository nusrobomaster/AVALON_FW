################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.c 

OBJS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.o 

C_DEPS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/%.o Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/%.su Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/%.cyclo: ../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/%.c Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/System/stm_utils/Inc" -I"C:/Users/ybakk/STM32CubeIDE/workspace_1.12.0/supercap_maybe/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-Common-2f-src-2f-transform_tests

clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-Common-2f-src-2f-transform_tests:
	-$(RM) ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_family_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/cfft_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/dct4_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_fast_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/rfft_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_test_group.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/transform_tests/transform_tests_common_data.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-Common-2f-src-2f-transform_tests

