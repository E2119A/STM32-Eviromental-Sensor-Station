################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bsp/display/fonts.c \
../Core/Src/bsp/display/ssd1306.c 

OBJS += \
./Core/Src/bsp/display/fonts.o \
./Core/Src/bsp/display/ssd1306.o 

C_DEPS += \
./Core/Src/bsp/display/fonts.d \
./Core/Src/bsp/display/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/bsp/display/%.o Core/Src/bsp/display/%.su Core/Src/bsp/display/%.cyclo: ../Core/Src/bsp/display/%.c Core/Src/bsp/display/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Core/Inc/app -I../Core/Inc/bsp -I../Core/Inc/system -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-bsp-2f-display

clean-Core-2f-Src-2f-bsp-2f-display:
	-$(RM) ./Core/Src/bsp/display/fonts.cyclo ./Core/Src/bsp/display/fonts.d ./Core/Src/bsp/display/fonts.o ./Core/Src/bsp/display/fonts.su ./Core/Src/bsp/display/ssd1306.cyclo ./Core/Src/bsp/display/ssd1306.d ./Core/Src/bsp/display/ssd1306.o ./Core/Src/bsp/display/ssd1306.su

.PHONY: clean-Core-2f-Src-2f-bsp-2f-display

