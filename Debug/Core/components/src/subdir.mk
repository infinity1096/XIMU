################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/components/src/dataProcessing.c \
../Core/components/src/gnss.c \
../Core/components/src/i2c.c \
../Core/components/src/led.c \
../Core/components/src/mpu9250.c \
../Core/components/src/ms5611.c 

OBJS += \
./Core/components/src/dataProcessing.o \
./Core/components/src/gnss.o \
./Core/components/src/i2c.o \
./Core/components/src/led.o \
./Core/components/src/mpu9250.o \
./Core/components/src/ms5611.o 

C_DEPS += \
./Core/components/src/dataProcessing.d \
./Core/components/src/gnss.d \
./Core/components/src/i2c.d \
./Core/components/src/led.d \
./Core/components/src/mpu9250.d \
./Core/components/src/ms5611.d 


# Each subdirectory must supply rules for building sources it contributes
Core/components/src/dataProcessing.o: ../Core/components/src/dataProcessing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../USB_DEVICE/Target -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/ros_lib" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/dataProcessing.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/gnss.o: ../Core/components/src/gnss.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../USB_DEVICE/Target -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/ros_lib" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/gnss.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/i2c.o: ../Core/components/src/i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../USB_DEVICE/Target -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/ros_lib" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/i2c.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/led.o: ../Core/components/src/led.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../USB_DEVICE/Target -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/ros_lib" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/led.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/mpu9250.o: ../Core/components/src/mpu9250.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../USB_DEVICE/Target -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/ros_lib" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/mpu9250.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/ms5611.o: ../Core/components/src/ms5611.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../USB_DEVICE/Target -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/ros_lib" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/ms5611.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

