################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/states/state_Menu_principal.cpp \
../src/states/state_Menu_pwm.cpp \
../src/states/state_Menu_servo.cpp \
../src/states/state_analogRead.cpp \
../src/states/state_blink.cpp \
../src/states/state_hardinit.cpp \
../src/states/state_pwm_0_255.cpp \
../src/states/state_pwm_0_5.cpp \
../src/states/state_pwm_pc.cpp \
../src/states/state_servo_micros.cpp \
../src/states/state_servo_selecter1.cpp \
../src/states/state_servo_selecter2.cpp 

OBJS += \
./src/states/state_Menu_principal.o \
./src/states/state_Menu_pwm.o \
./src/states/state_Menu_servo.o \
./src/states/state_analogRead.o \
./src/states/state_blink.o \
./src/states/state_hardinit.o \
./src/states/state_pwm_0_255.o \
./src/states/state_pwm_0_5.o \
./src/states/state_pwm_pc.o \
./src/states/state_servo_micros.o \
./src/states/state_servo_selecter1.o \
./src/states/state_servo_selecter2.o 

CPP_DEPS += \
./src/states/state_Menu_principal.d \
./src/states/state_Menu_pwm.d \
./src/states/state_Menu_servo.d \
./src/states/state_analogRead.d \
./src/states/state_blink.d \
./src/states/state_hardinit.d \
./src/states/state_pwm_0_255.d \
./src/states/state_pwm_0_5.d \
./src/states/state_pwm_pc.d \
./src/states/state_servo_micros.d \
./src/states/state_servo_selecter1.d \
./src/states/state_servo_selecter2.d 


# Each subdirectory must supply rules for building sources it contributes
src/states/%.o: ../src/states/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/Uno/src" -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Encoder" -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Wire" -I"/media/fabien/Data/Documents/Robots-master/static/core/arduino/libraries/Servo" -I"/media/fabien/Data/Documents/Robots-master/static/tools/tester/src/libs" -I"/media/fabien/Data/Documents/Robots-master/static/tools/tester/src/states" -I"/media/fabien/Data/Documents/Robots-master/static/tools/tester/src" -DARCH_328P_ARDUINO -DARDUINO=100 -DCORE_NUM_INTERRUPT=2 -DCORE_INT0_PIN=2 -DCORE_INT1_PIN=3 -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


