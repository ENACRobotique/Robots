################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/states/subdir.mk
-include src/subdir.mk
-include arduinolib/Wire/utility/subdir.mk
-include arduinolib/Wire/subdir.mk
-include arduinolib/Servo/subdir.mk
-include arduinolib/LiquidCrystal/subdir.mk
-include arduinolib/Encoder/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
tester.lss \

FLASH_IMAGE += \
tester.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: tester.elf secondary-outputs

dependents:
	-cd /media/fabien/Data/Documents/Robots-master/static/core/arduino/Uno/328P_16MHz && $(MAKE) all

# Tool invocations
tester.elf: $(OBJS) $(USER_OBJS) /media/fabien/Data/Documents/Robots-master/static/core/arduino/Uno/328P_16MHz/libarduinoUno_core.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,tester.map,--cref -mrelax -Wl,--gc-sections -Wl,--gc-sections -L"/media/fabien/Data/Documents/Robots-master/static/core/arduino/Uno/328P_16MHz" -mmcu=atmega328p -o "tester.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

tester.lss: tester.elf
	@echo 'Invoking: AVR Create Extended Listing'
	avr-objdump -h -S tester.elf  >"tester.lss"
	@echo 'Finished building: $@'
	@echo ' '

tester.hex: tester.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex tester.elf  "tester.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: tester.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p tester.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) tester.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
/media/fabien/Data/Documents/Robots-master/static/core/arduino/Uno/328P_16MHz/libarduinoUno_core.a:

-include ../makefile.targets
