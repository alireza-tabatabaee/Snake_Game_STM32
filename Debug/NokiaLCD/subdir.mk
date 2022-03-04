################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NokiaLCD/stm32f4_pcd8544.c 

OBJS += \
./NokiaLCD/stm32f4_pcd8544.o 

C_DEPS += \
./NokiaLCD/stm32f4_pcd8544.d 


# Each subdirectory must supply rules for building sources it contributes
NokiaLCD/stm32f4_pcd8544.o: ../NokiaLCD/stm32f4_pcd8544.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DDEBUG -c -I"C:/Users/Alireza/Documents/_MCU/Snake/NokiaLCD" -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"NokiaLCD/stm32f4_pcd8544.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

