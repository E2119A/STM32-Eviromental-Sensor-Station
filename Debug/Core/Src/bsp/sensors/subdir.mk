################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bsp/sensors/bmp280.c 

OBJS += \
./Core/Src/bsp/sensors/bmp280.o 

C_DEPS += \
./Core/Src/bsp/sensors/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/bsp/sensors/%.o Core/Src/bsp/sensors/%.su Core/Src/bsp/sensors/%.cyclo: ../Core/Src/bsp/sensors/%.c Core/Src/bsp/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Core/Inc/app -I../Core/Inc/bsp -I../Core/Inc/system -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-bsp-2f-sensors

clean-Core-2f-Src-2f-bsp-2f-sensors:
	-$(RM) ./Core/Src/bsp/sensors/bmp280.cyclo ./Core/Src/bsp/sensors/bmp280.d ./Core/Src/bsp/sensors/bmp280.o ./Core/Src/bsp/sensors/bmp280.su

.PHONY: clean-Core-2f-Src-2f-bsp-2f-sensors

