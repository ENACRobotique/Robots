################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/seb/RobotOfficiel/Robots/static/core/linux/libraries/Millis/millis.c 

OBJS += \
./lib/Millis/millis.o 

C_DEPS += \
./lib/Millis/millis.d 


# Each subdirectory must supply rules for building sources it contributes
lib/Millis/millis.o: /home/seb/RobotOfficiel/Robots/static/core/linux/libraries/Millis/millis.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DARCH_X86_LINUX -DARCH_LITTLE_ENDIAN -I"/home/seb/RobotOfficiel/Robots/static/communication/botNet" -I"/home/seb/RobotOfficiel/Robots/static/core/linux/libraries/Millis" -I"/home/seb/RobotOfficiel/Robots/network_config" -I"/home/seb/RobotOfficiel/Robots/static/tools/libraries/Timeout" -I"/home/seb/RobotOfficiel/Robots/static/communication/UART_framing" -I"/home/seb/RobotOfficiel/Robots/static/communication/Xbee_API" -I"/home/seb/RobotOfficiel/Robots/primary_robot/linux_ia" -O2 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


