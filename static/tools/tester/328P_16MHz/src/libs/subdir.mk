################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/libs/lib_line.cpp \
../src/libs/lib_motor.cpp \
../src/libs/lib_move.cpp \
../src/libs/lib_odo.cpp \
../src/libs/lib_radar.cpp \
../src/libs/lib_us.cpp \
../src/libs/lib_wall.cpp 

C_SRCS += \
../src/libs/sharp_2d120x.c 

OBJS += \
./src/libs/lib_line.o \
./src/libs/lib_motor.o \
./src/libs/lib_move.o \
./src/libs/lib_odo.o \
./src/libs/lib_radar.o \
./src/libs/lib_us.o \
./src/libs/lib_wall.o \
./src/libs/sharp_2d120x.o 

C_DEPS += \
./src/libs/sharp_2d120x.d 

CPP_DEPS += \
./src/libs/lib_line.d \
./src/libs/lib_motor.d \
./src/libs/lib_move.d \
./src/libs/lib_odo.d \
./src/libs/lib_radar.d \
./src/libs/lib_us.d \
./src/libs/lib_wall.d 


# Each subdirectory must supply rules for building sources it contributes
src/libs/%.o: ../src/libs/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/mnt/data/Documents/Robots-master/static/core/arduino/Uno/src" -I"/mnt/data/Documents/Robots-master/static/core/arduino/libraries/Wire" -I"/mnt/data/Documents/Robots-master/static/core/arduino/libraries/Servo" -I"/mnt/data/Documents/Robots-master/secondary_robot/src/libs" -I"/mnt/data/Documents/Robots-master/secondary_robot/src/states" -I"/mnt/data/Documents/Robots-master/secondary_robot/src" -DARCH_328P_ARDUINO -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/libs/%.o: ../src/libs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/mnt/data/Documents/Robots-master/static/core/arduino/Uno/src" -I"/mnt/data/Documents/Robots-master/static/core/arduino/libraries/Wire" -I"/mnt/data/Documents/Robots-master/static/core/arduino/libraries/Servo" -I"/mnt/data/Documents/Robots-master/secondary_robot/src/libs" -I"/mnt/data/Documents/Robots-master/secondary_robot/src/states" -I"/mnt/data/Documents/Robots-master/secondary_robot/src" -DARCH_328P_ARDUINO -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


