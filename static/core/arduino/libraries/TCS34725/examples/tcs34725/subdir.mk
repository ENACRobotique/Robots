################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
PDE_SRCS += \
/home/guilhem/Documents/Club_Robot/static/core/arduino/libraries/TCS34725/examples/tcs34725/tcs34725.pde 

PDE_DEPS += \
./lib/TCS34725/examples/tcs34725/tcs34725.d 

OBJS += \
./lib/TCS34725/examples/tcs34725/tcs34725.o 


# Each subdirectory must supply rules for building sources it contributes
lib/TCS34725/examples/tcs34725/tcs34725.o: /home/guilhem/Documents/Club_Robot/static/core/arduino/libraries/TCS34725/examples/tcs34725/tcs34725.pde
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/guilhem/Documents/Club_Robot/static/core/arduino/Uno/src" -I"/home/guilhem/Documents/Club_Robot/static/core/arduino/libraries/Wire" -I"/home/guilhem/Documents/Club_Robot/static/core/arduino/libraries/TCS34725" -DARCH_LITTLE_ENDIAN -DARCH_328P_ARDUINO -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


