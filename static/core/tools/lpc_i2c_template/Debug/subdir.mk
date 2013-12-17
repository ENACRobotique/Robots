################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main.c 

OBJS += \
./main.o 

C_DEPS += \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/tools/lpc_i2c_template" -O0 -g3 -Wall -c -fno-common -mcpu=arm7tdmi -march=armv4t -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


