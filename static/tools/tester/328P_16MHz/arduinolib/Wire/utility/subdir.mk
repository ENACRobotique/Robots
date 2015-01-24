################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Wire/utility/twi.c 

OBJS += \
./arduinolib/Wire/utility/twi.o 

C_DEPS += \
./arduinolib/Wire/utility/twi.d 


# Each subdirectory must supply rules for building sources it contributes
arduinolib/Wire/utility/twi.o: /media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Wire/utility/twi.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/Uno/src" -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Wire" -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Servo" -I"/media/fabien/Data/Documents/Robots-master/static/tools/tester/src/libs" -I"/media/fabien/Data/Documents/Robots-master/static/tools/tester/src/states" -I"/media/fabien/Data/Documents/Robots-master/static/tools/tester/src" -DARCH_328P_ARDUINO -DARDUINO=100 -DCORE_NUM_INTERRUPT=2 -DCORE_INT0_PIN=2 -DCORE_INT1_PIN=3 -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


