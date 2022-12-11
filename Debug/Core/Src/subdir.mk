################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c 

CPP_SRCS += \
../Core/Src/AS5048B.cpp \
../Core/Src/BLDCMotor.cpp \
../Core/Src/CircBuffer.cpp \
../Core/Src/MPU9050.cpp \
../Core/Src/PID.cpp \
../Core/Src/main.cpp \
../Core/Src/serialPort.cpp \
../Core/Src/stm32f4xx_hal_msp.cpp \
../Core/Src/stm32f4xx_it.cpp \
../Core/Src/sysmem.cpp 

C_DEPS += \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/AS5048B.o \
./Core/Src/BLDCMotor.o \
./Core/Src/CircBuffer.o \
./Core/Src/MPU9050.o \
./Core/Src/PID.o \
./Core/Src/main.o \
./Core/Src/serialPort.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

CPP_DEPS += \
./Core/Src/AS5048B.d \
./Core/Src/BLDCMotor.d \
./Core/Src/CircBuffer.d \
./Core/Src/MPU9050.d \
./Core/Src/PID.d \
./Core/Src/main.d \
./Core/Src/serialPort.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AS5048B.d ./Core/Src/AS5048B.o ./Core/Src/AS5048B.su ./Core/Src/BLDCMotor.d ./Core/Src/BLDCMotor.o ./Core/Src/BLDCMotor.su ./Core/Src/CircBuffer.d ./Core/Src/CircBuffer.o ./Core/Src/CircBuffer.su ./Core/Src/MPU9050.d ./Core/Src/MPU9050.o ./Core/Src/MPU9050.su ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/serialPort.d ./Core/Src/serialPort.o ./Core/Src/serialPort.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

