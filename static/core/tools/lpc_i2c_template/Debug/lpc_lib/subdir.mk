################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/eint.c \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/i2c.c \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/i2c0.c \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/pwm.c \
/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/sys_time.c 

OBJS += \
./lpc_lib/eint.o \
./lpc_lib/i2c.o \
./lpc_lib/i2c0.o \
./lpc_lib/pwm.o \
./lpc_lib/sys_time.o 

C_DEPS += \
./lpc_lib/eint.d \
./lpc_lib/i2c.d \
./lpc_lib/i2c0.d \
./lpc_lib/pwm.d \
./lpc_lib/sys_time.d 


# Each subdirectory must supply rules for building sources it contributes
lpc_lib/eint.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/eint.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/tools/lpc_i2c_template" -O0 -g3 -Wall -c -fno-common -mcpu=arm7tdmi -march=armv4t -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lpc_lib/i2c.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/tools/lpc_i2c_template" -O0 -g3 -Wall -c -fno-common -mcpu=arm7tdmi -march=armv4t -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lpc_lib/i2c0.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/i2c0.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/tools/lpc_i2c_template" -O0 -g3 -Wall -c -fno-common -mcpu=arm7tdmi -march=armv4t -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lpc_lib/pwm.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/pwm.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/tools/lpc_i2c_template" -O0 -g3 -Wall -c -fno-common -mcpu=arm7tdmi -march=armv4t -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lpc_lib/sys_time.o: /home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib/sys_time.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc2148" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/lpc21xx/lpc_lib" -I"/home/ludo6431/Documents/ENAC/ClubRobot/github/static/core/tools/lpc_i2c_template" -O0 -g3 -Wall -c -fno-common -mcpu=arm7tdmi -march=armv4t -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


