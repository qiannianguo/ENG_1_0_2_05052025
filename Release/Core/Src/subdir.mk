################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/EVE_API.c \
../Core/Src/EVE_HAL.c \
../Core/Src/EVE_MCU_STM32.c \
../Core/Src/MCU_init.c \
../Core/Src/STM_HAL.c \
../Core/Src/eve_calibrate.c \
../Core/Src/functions.c \
../Core/Src/io_calls.c \
../Core/Src/main.c \
../Core/Src/main_tasks.c \
../Core/Src/screen_helper.c \
../Core/Src/screens.c \
../Core/Src/stm32l0xx_hal_msp.c \
../Core/Src/stm32l0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l0xx.c \
../Core/Src/testfuntions.c 

OBJS += \
./Core/Src/EVE_API.o \
./Core/Src/EVE_HAL.o \
./Core/Src/EVE_MCU_STM32.o \
./Core/Src/MCU_init.o \
./Core/Src/STM_HAL.o \
./Core/Src/eve_calibrate.o \
./Core/Src/functions.o \
./Core/Src/io_calls.o \
./Core/Src/main.o \
./Core/Src/main_tasks.o \
./Core/Src/screen_helper.o \
./Core/Src/screens.o \
./Core/Src/stm32l0xx_hal_msp.o \
./Core/Src/stm32l0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l0xx.o \
./Core/Src/testfuntions.o 

C_DEPS += \
./Core/Src/EVE_API.d \
./Core/Src/EVE_HAL.d \
./Core/Src/EVE_MCU_STM32.d \
./Core/Src/MCU_init.d \
./Core/Src/STM_HAL.d \
./Core/Src/eve_calibrate.d \
./Core/Src/functions.d \
./Core/Src/io_calls.d \
./Core/Src/main.d \
./Core/Src/main_tasks.d \
./Core/Src/screen_helper.d \
./Core/Src/screens.d \
./Core/Src/stm32l0xx_hal_msp.d \
./Core/Src/stm32l0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l0xx.d \
./Core/Src/testfuntions.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L073xx -c -I../Core/Inc -I"C:/Users/jeffg_guo/STM32Cube/Repository/STM32Cube_FW_L0_V1.12.3/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/Users/jeffg_guo/STM32Cube/Repository/STM32Cube_FW_L0_V1.12.3/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/jeffg_guo/STM32Cube/Repository/STM32Cube_FW_L0_V1.12.3/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"C:/Users/jeffg_guo/STM32Cube/Repository/STM32Cube_FW_L0_V1.12.3/Drivers/CMSIS/Include" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/EVE_API.cyclo ./Core/Src/EVE_API.d ./Core/Src/EVE_API.o ./Core/Src/EVE_API.su ./Core/Src/EVE_HAL.cyclo ./Core/Src/EVE_HAL.d ./Core/Src/EVE_HAL.o ./Core/Src/EVE_HAL.su ./Core/Src/EVE_MCU_STM32.cyclo ./Core/Src/EVE_MCU_STM32.d ./Core/Src/EVE_MCU_STM32.o ./Core/Src/EVE_MCU_STM32.su ./Core/Src/MCU_init.cyclo ./Core/Src/MCU_init.d ./Core/Src/MCU_init.o ./Core/Src/MCU_init.su ./Core/Src/STM_HAL.cyclo ./Core/Src/STM_HAL.d ./Core/Src/STM_HAL.o ./Core/Src/STM_HAL.su ./Core/Src/eve_calibrate.cyclo ./Core/Src/eve_calibrate.d ./Core/Src/eve_calibrate.o ./Core/Src/eve_calibrate.su ./Core/Src/functions.cyclo ./Core/Src/functions.d ./Core/Src/functions.o ./Core/Src/functions.su ./Core/Src/io_calls.cyclo ./Core/Src/io_calls.d ./Core/Src/io_calls.o ./Core/Src/io_calls.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/main_tasks.cyclo ./Core/Src/main_tasks.d ./Core/Src/main_tasks.o ./Core/Src/main_tasks.su ./Core/Src/screen_helper.cyclo ./Core/Src/screen_helper.d ./Core/Src/screen_helper.o ./Core/Src/screen_helper.su ./Core/Src/screens.cyclo ./Core/Src/screens.d ./Core/Src/screens.o ./Core/Src/screens.su ./Core/Src/stm32l0xx_hal_msp.cyclo ./Core/Src/stm32l0xx_hal_msp.d ./Core/Src/stm32l0xx_hal_msp.o ./Core/Src/stm32l0xx_hal_msp.su ./Core/Src/stm32l0xx_it.cyclo ./Core/Src/stm32l0xx_it.d ./Core/Src/stm32l0xx_it.o ./Core/Src/stm32l0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l0xx.cyclo ./Core/Src/system_stm32l0xx.d ./Core/Src/system_stm32l0xx.o ./Core/Src/system_stm32l0xx.su ./Core/Src/testfuntions.cyclo ./Core/Src/testfuntions.d ./Core/Src/testfuntions.o ./Core/Src/testfuntions.su

.PHONY: clean-Core-2f-Src

