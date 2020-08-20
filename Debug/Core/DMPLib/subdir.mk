################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/DMPLib/inv_mpu.c \
../Core/DMPLib/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Core/DMPLib/inv_mpu.o \
./Core/DMPLib/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Core/DMPLib/inv_mpu.d \
./Core/DMPLib/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/DMPLib/inv_mpu.o: ../Core/DMPLib/inv_mpu.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DMPLib/inv_mpu.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/DMPLib/inv_mpu_dmp_motion_driver.o: ../Core/DMPLib/inv_mpu_dmp_motion_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DMPLib/inv_mpu_dmp_motion_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

