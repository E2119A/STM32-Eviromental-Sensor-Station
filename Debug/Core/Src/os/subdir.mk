################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/os/freertos.c 

OBJS += \
./Core/Src/os/freertos.o 

C_DEPS += \
./Core/Src/os/freertos.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/os/%.o Core/Src/os/%.su Core/Src/os/%.cyclo: ../Core/Src/os/%.c Core/Src/os/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Core/Inc/app -I../Core/Inc/bsp -I../Core/Inc/system -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-os

clean-Core-2f-Src-2f-os:
	-$(RM) ./Core/Src/os/freertos.cyclo ./Core/Src/os/freertos.d ./Core/Src/os/freertos.o ./Core/Src/os/freertos.su

.PHONY: clean-Core-2f-Src-2f-os

