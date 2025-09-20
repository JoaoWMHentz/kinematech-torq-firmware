################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/math/svpwm.c 

C_DEPS += \
./Core/Src/math/svpwm.d 

OBJS += \
./Core/Src/math/svpwm.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/math/%.o Core/Src/math/%.su Core/Src/math/%.cyclo: ../Core/Src/math/%.c Core/Src/math/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-math

clean-Core-2f-Src-2f-math:
	-$(RM) ./Core/Src/math/svpwm.cyclo ./Core/Src/math/svpwm.d ./Core/Src/math/svpwm.o ./Core/Src/math/svpwm.su

.PHONY: clean-Core-2f-Src-2f-math

