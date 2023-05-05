################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../WiFi/es_wifi_io.c \
../WiFi/wifi.c 

OBJS += \
./WiFi/es_wifi_io.o \
./WiFi/wifi.o 

C_DEPS += \
./WiFi/es_wifi_io.d \
./WiFi/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
WiFi/%.o WiFi/%.su: ../WiFi/%.c WiFi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -DGENERICMQTT -DMSG_ERROR -DUSE_CLEAR_TIMEDATE -DUSE_TIMEDATE -DUSE_WIFI -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/B-L475E-IOT01" -I"../Drivers/BSP/Components/es_wifi" -I"../Drivers/BSP/Components/Common" -I../WiFi -I"../Common/Shared/Inc" -I"../Middleware/cJSON" -I"../Middleware/MQTTPacket" -I"../Middleware/MQTTClient-C" -I"../Common/GenericMQTT/Inc" -I"../Middleware/mbedTLS/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-WiFi

clean-WiFi:
	-$(RM) ./WiFi/es_wifi_io.d ./WiFi/es_wifi_io.o ./WiFi/es_wifi_io.su ./WiFi/wifi.d ./WiFi/wifi.o ./WiFi/wifi.su

.PHONY: clean-WiFi

