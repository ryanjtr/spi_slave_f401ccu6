################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SPI_SLAVE/slave_reg.c 

OBJS += \
./SPI_SLAVE/slave_reg.o 

C_DEPS += \
./SPI_SLAVE/slave_reg.d 


# Each subdirectory must supply rules for building sources it contributes
SPI_SLAVE/%.o SPI_SLAVE/%.su SPI_SLAVE/%.cyclo: ../SPI_SLAVE/%.c SPI_SLAVE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F401xC -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"F:/STM32project/modelization/spi_slave_stm32f401ccu6/SPI_SLAVE" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SPI_SLAVE

clean-SPI_SLAVE:
	-$(RM) ./SPI_SLAVE/slave_reg.cyclo ./SPI_SLAVE/slave_reg.d ./SPI_SLAVE/slave_reg.o ./SPI_SLAVE/slave_reg.su

.PHONY: clean-SPI_SLAVE

