################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
F446RE\ RTOS\ HarmMotionSynth.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F446RE\ RTOS\ HarmMotionSynth.list \

OBJCOPY_BIN += \
F446RE\ RTOS\ HarmMotionSynth.bin \


# All Target
all: F446RE\ RTOS\ HarmMotionSynth.elf secondary-outputs

# Tool invocations
F446RE\ RTOS\ HarmMotionSynth.elf: $(OBJS) $(USER_OBJS) C:\Users\zakke\STM32CubeIDE\workspace_1.3.0\F446RE\ RTOS\ synth\STM32F446RETX_FLASH.ld
	arm-none-eabi-g++ -o "F446RE RTOS HarmMotionSynth.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\zakke\STM32CubeIDE\workspace_1.3.0\F446RE RTOS synth\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F446RE RTOS HarmMotionSynth.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F446RE\ RTOS\ HarmMotionSynth.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F446RE RTOS HarmMotionSynth.list"
	@echo 'Finished building: $@'
	@echo ' '

F446RE\ RTOS\ HarmMotionSynth.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "F446RE RTOS HarmMotionSynth.bin"
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
