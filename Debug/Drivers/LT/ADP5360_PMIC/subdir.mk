################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LT/ADP5360_PMIC/adp5360.c 

OBJS += \
./Drivers/LT/ADP5360_PMIC/adp5360.o 

C_DEPS += \
./Drivers/LT/ADP5360_PMIC/adp5360.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LT/ADP5360_PMIC/%.o Drivers/LT/ADP5360_PMIC/%.su Drivers/LT/ADP5360_PMIC/%.cyclo: ../Drivers/LT/ADP5360_PMIC/%.c Drivers/LT/ADP5360_PMIC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB10xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/ADP5360_PMIC" -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/LT_App" -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/M24C64_EEPROM" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LT-2f-ADP5360_PMIC

clean-Drivers-2f-LT-2f-ADP5360_PMIC:
	-$(RM) ./Drivers/LT/ADP5360_PMIC/adp5360.cyclo ./Drivers/LT/ADP5360_PMIC/adp5360.d ./Drivers/LT/ADP5360_PMIC/adp5360.o ./Drivers/LT/ADP5360_PMIC/adp5360.su

.PHONY: clean-Drivers-2f-LT-2f-ADP5360_PMIC

