################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bsp/sd/sd_benchmark.c \
../Core/Src/bsp/sd/sd_diskio_spi.c \
../Core/Src/bsp/sd/sd_functions.c \
../Core/Src/bsp/sd/sd_spi.c 

OBJS += \
./Core/Src/bsp/sd/sd_benchmark.o \
./Core/Src/bsp/sd/sd_diskio_spi.o \
./Core/Src/bsp/sd/sd_functions.o \
./Core/Src/bsp/sd/sd_spi.o 

C_DEPS += \
./Core/Src/bsp/sd/sd_benchmark.d \
./Core/Src/bsp/sd/sd_diskio_spi.d \
./Core/Src/bsp/sd/sd_functions.d \
./Core/Src/bsp/sd/sd_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/bsp/sd/%.o Core/Src/bsp/sd/%.su Core/Src/bsp/sd/%.cyclo: ../Core/Src/bsp/sd/%.c Core/Src/bsp/sd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Core/Inc/app -I../Core/Inc/bsp -I../Core/Inc/system -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-bsp-2f-sd

clean-Core-2f-Src-2f-bsp-2f-sd:
	-$(RM) ./Core/Src/bsp/sd/sd_benchmark.cyclo ./Core/Src/bsp/sd/sd_benchmark.d ./Core/Src/bsp/sd/sd_benchmark.o ./Core/Src/bsp/sd/sd_benchmark.su ./Core/Src/bsp/sd/sd_diskio_spi.cyclo ./Core/Src/bsp/sd/sd_diskio_spi.d ./Core/Src/bsp/sd/sd_diskio_spi.o ./Core/Src/bsp/sd/sd_diskio_spi.su ./Core/Src/bsp/sd/sd_functions.cyclo ./Core/Src/bsp/sd/sd_functions.d ./Core/Src/bsp/sd/sd_functions.o ./Core/Src/bsp/sd/sd_functions.su ./Core/Src/bsp/sd/sd_spi.cyclo ./Core/Src/bsp/sd/sd_spi.d ./Core/Src/bsp/sd/sd_spi.o ./Core/Src/bsp/sd/sd_spi.su

.PHONY: clean-Core-2f-Src-2f-bsp-2f-sd

