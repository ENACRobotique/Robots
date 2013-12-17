################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148/crt.s \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148/ime.s 

OBJS += \
./lpc2148/crt.o \
./lpc2148/ime.o 


# Each subdirectory must supply rules for building sources it contributes
lpc2148/crt.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148/crt.s
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	arm-none-eabi-as -mcpu=arm7tdmi -march=armv4t -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lpc2148/ime.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148/ime.s
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	arm-none-eabi-as -mcpu=arm7tdmi -march=armv4t -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


