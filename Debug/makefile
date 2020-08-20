################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/components/src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/DMPLib/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
XIMU.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
XIMU.list \

OBJCOPY_BIN += \
XIMU.bin \


# All Target
all: XIMU.elf secondary-outputs

# Tool invocations
XIMU.elf: $(OBJS) $(USER_OBJS) C:\Users\yuche\STM32CubeIDE\workspace_1.3.0\XIMU\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "XIMU.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\yuche\STM32CubeIDE\workspace_1.3.0\XIMU\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="XIMU.map" -Wl,--gc-sections -static -L"C:\Users\yuche\STM32CubeIDE\workspace_1.3.0\XIMU\Core\libraries" --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

XIMU.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "XIMU.list"
	@echo 'Finished building: $@'
	@echo ' '

XIMU.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "XIMU.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
