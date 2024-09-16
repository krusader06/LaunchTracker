################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LT/LT_App/LT_App.c 

OBJS += \
./Drivers/LT/LT_App/LT_App.o 

C_DEPS += \
./Drivers/LT/LT_App/LT_App.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LT/LT_App/%.o Drivers/LT/LT_App/%.su Drivers/LT/LT_App/%.cyclo: ../Drivers/LT/LT_App/%.c Drivers/LT/LT_App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB10xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/ADP5360_PMIC" -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/KX134_ACCEL" -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/LT_App" -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/M24C64_EEPROM" -I"C:/Users/Colton/STM32CubeIDE/workspace_1.15.1/LaunchTracker/Drivers/LT/SPL07_PRESS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LT-2f-LT_App

clean-Drivers-2f-LT-2f-LT_App:
	-$(RM) ./Drivers/LT/LT_App/LT_App.cyclo ./Drivers/LT/LT_App/LT_App.d ./Drivers/LT/LT_App/LT_App.o ./Drivers/LT/LT_App/LT_App.su

.PHONY: clean-Drivers-2f-LT-2f-LT_App

