################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.c 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.d 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-WindowFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-WindowFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/WindowFunctions/WindowFunctions.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-WindowFunctions

