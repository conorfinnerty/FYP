################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/cJSON/cJSON.c \
../Middleware/cJSON/cJSON_Utils.c 

OBJS += \
./Middleware/cJSON/cJSON.o \
./Middleware/cJSON/cJSON_Utils.o 

C_DEPS += \
./Middleware/cJSON/cJSON.d \
./Middleware/cJSON/cJSON_Utils.d 


# Each subdirectory must supply rules for building sources it contributes
Middleware/cJSON/%.o Middleware/cJSON/%.su: ../Middleware/cJSON/%.c Middleware/cJSON/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -DGENERICMQTT -DMSG_ERROR -DUSE_CLEAR_TIMEDATE -DUSE_TIMEDATE -DUSE_WIFI -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/B-L475E-IOT01" -I"../Drivers/BSP/Components/es_wifi" -I"../Drivers/BSP/Components/Common" -I../WiFi -I"../Common/Shared/Inc" -I"../Middleware/cJSON" -I"../Middleware/MQTTPacket" -I"../Middleware/MQTTClient-C" -I"../Common/GenericMQTT/Inc" -I"../Middleware/mbedTLS/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middleware-2f-cJSON

clean-Middleware-2f-cJSON:
	-$(RM) ./Middleware/cJSON/cJSON.d ./Middleware/cJSON/cJSON.o ./Middleware/cJSON/cJSON.su ./Middleware/cJSON/cJSON_Utils.d ./Middleware/cJSON/cJSON_Utils.o ./Middleware/cJSON/cJSON_Utils.su

.PHONY: clean-Middleware-2f-cJSON

