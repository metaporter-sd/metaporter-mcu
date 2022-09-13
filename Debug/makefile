################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := metaporter-mcu
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
metaporter-mcu.elf \

MAP_FILES += \
metaporter-mcu.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
metaporter-mcu.list \


# All Target
all: main-build

# Main-build Target
main-build: metaporter-mcu.elf secondary-outputs

dependents:
	-cd /Users/manavbhasin/STM32CubeIDE/workspace_1.10.1/metaporter-mcu/Release && $(MAKE) all

# Tool invocations
metaporter-mcu.elf metaporter-mcu.map: $(OBJS) $(USER_OBJS) /Users/manavbhasin/STM32CubeIDE/workspace_1.10.1/metaporter-mcu/Release/metaporter-mcu.elf /Users/manavbhasin/STM32CubeIDE/workspace_1.10.1/metaporter-mcu/STM32F091RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "metaporter-mcu.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/Users/manavbhasin/STM32CubeIDE/workspace_1.10.1/metaporter-mcu/STM32F091RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="metaporter-mcu.map" -Wl,--gc-sections -static -L"/Users/manavbhasin/STM32CubeIDE/workspace_1.10.1/metaporter-mcu/Core/Src" -L"C:\Users\12197\STM32CubeIDE\workspace_1.10.1\metaporter-mcu\Core\Src" --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

metaporter-mcu.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "metaporter-mcu.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout metaporter-mcu.elf metaporter-mcu.list metaporter-mcu.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified
/Users/manavbhasin/STM32CubeIDE/workspace_1.10.1/metaporter-mcu/Release/metaporter-mcu.elf:

-include ../makefile.targets
