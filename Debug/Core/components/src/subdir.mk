################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/components/src/ESKF.c \
../Core/components/src/armMathUtils.c \
../Core/components/src/dataProcessing.c \
../Core/components/src/gnss.c \
../Core/components/src/i2c.c \
../Core/components/src/led.c \
../Core/components/src/mpu9250.c \
../Core/components/src/ms5611.c \
../Core/components/src/quaternion.c \
../Core/components/src/systick.c 

OBJS += \
./Core/components/src/ESKF.o \
./Core/components/src/armMathUtils.o \
./Core/components/src/dataProcessing.o \
./Core/components/src/gnss.o \
./Core/components/src/i2c.o \
./Core/components/src/led.o \
./Core/components/src/mpu9250.o \
./Core/components/src/ms5611.o \
./Core/components/src/quaternion.o \
./Core/components/src/systick.o 

C_DEPS += \
./Core/components/src/ESKF.d \
./Core/components/src/armMathUtils.d \
./Core/components/src/dataProcessing.d \
./Core/components/src/gnss.d \
./Core/components/src/i2c.d \
./Core/components/src/led.d \
./Core/components/src/mpu9250.d \
./Core/components/src/ms5611.d \
./Core/components/src/quaternion.d \
./Core/components/src/systick.d 


# Each subdirectory must supply rules for building sources it contributes
Core/components/src/ESKF.o: ../Core/components/src/ESKF.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/ESKF.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/armMathUtils.o: ../Core/components/src/armMathUtils.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/armMathUtils.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/dataProcessing.o: ../Core/components/src/dataProcessing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/dataProcessing.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/gnss.o: ../Core/components/src/gnss.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/gnss.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/i2c.o: ../Core/components/src/i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/i2c.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/led.o: ../Core/components/src/led.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/led.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/mpu9250.o: ../Core/components/src/mpu9250.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/mpu9250.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/ms5611.o: ../Core/components/src/ms5611.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/ms5611.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/quaternion.o: ../Core/components/src/quaternion.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/quaternion.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/components/src/systick.o: ../Core/components/src/systick.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -DARM_MATH_CM3 -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/DMPLib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/yuche/STM32CubeIDE/workspace_1.3.0/XIMU/Core/components/inc" -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/components/src/systick.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

