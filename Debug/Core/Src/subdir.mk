################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/diagnostics.c \
../Core/Src/gpio.c \
../Core/Src/hall_sensor.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_hal_timebase_tim.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usb_communication.c 

C_DEPS += \
./Core/Src/diagnostics.d \
./Core/Src/gpio.d \
./Core/Src/hall_sensor.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_hal_timebase_tim.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usb_communication.d 

OBJS += \
./Core/Src/diagnostics.o \
./Core/Src/gpio.o \
./Core/Src/hall_sensor.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_hal_timebase_tim.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usb_communication.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I/home/joaoh/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc -I/home/joaoh/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I/home/joaoh/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I/home/joaoh/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I/home/joaoh/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Device/ST/STM32G4xx/Include -I/home/joaoh/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Include -I/home/joaoh/STM32Cube/Repository//Packs/ARM/CMSIS/6.2.0/CMSIS/Core/Include/ -I/home/joaoh/STM32Cube/Repository//Packs/ARM/CMSIS-DSP/1.16.2/PrivateInclude/ -I/home/joaoh/STM32Cube/Repository//Packs/ARM/CMSIS-DSP/1.16.2/Include/ -I/home/joaoh/STM32Cube/Repository//Packs/ARM/CMSIS-DSP/1.16.2/Include -I/home/joaow/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc -I/home/joaow/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I/home/joaow/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I/home/joaow/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I/home/joaow/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Device/ST/STM32G4xx/Include -I/home/joaow/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Include -I/home/joaow/STM32Cube/Repository//Packs/ARM/CMSIS/6.2.0/CMSIS/Core/Include/ -I/home/joaow/STM32Cube/Repository//Packs/ARM/CMSIS-DSP/1.16.2/PrivateInclude/ -I/home/joaow/STM32Cube/Repository//Packs/ARM/CMSIS-DSP/1.16.2/Include/ -I/home/joaow/STM32Cube/Repository//Packs/ARM/CMSIS-DSP/1.16.2/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/diagnostics.cyclo ./Core/Src/diagnostics.d ./Core/Src/diagnostics.o ./Core/Src/diagnostics.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hall_sensor.cyclo ./Core/Src/hall_sensor.d ./Core/Src/hall_sensor.o ./Core/Src/hall_sensor.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_hal_timebase_tim.cyclo ./Core/Src/stm32g4xx_hal_timebase_tim.d ./Core/Src/stm32g4xx_hal_timebase_tim.o ./Core/Src/stm32g4xx_hal_timebase_tim.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usb_communication.cyclo ./Core/Src/usb_communication.d ./Core/Src/usb_communication.o ./Core/Src/usb_communication.su

.PHONY: clean-Core-2f-Src

