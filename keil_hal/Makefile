# Build GCC OPSIONAL (arm-none-eabi-gcc). Project utama = MDK-ARM/qbc_hinf.uvprojx (Keil).
# make            -> build/qbc_hinf.elf, .hex, .bin
TARGET = qbc_hinf
BUILD  = build
CC = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS = -DUSE_HAL_DRIVER -DSTM32F407xx
INCS = -ICore/Inc -IApp -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
       -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include
SRCS = $(wildcard Core/Src/*.c) $(wildcard App/*.c) $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)
ASRC = gcc/startup_stm32f407xx.s
CFLAGS = $(MCU) $(DEFS) $(INCS) -O2 -Wall -Wextra -Wno-unused-parameter -ffunction-sections -fdata-sections -std=c99
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -Tgcc/STM32F407VGTx_FLASH.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map
OBJS = $(addprefix $(BUILD)/,$(SRCS:.c=.o)) $(BUILD)/startup.o
all: $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@
$(BUILD)/startup.o: $(ASRC)
	@mkdir -p $(BUILD)
	$(CC) -c $(MCU) -x assembler-with-cpp $< -o $@
$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -lm -o $@
	$(SZ) $@
$(BUILD)/%.hex: $(BUILD)/%.elf
	$(CP) -O ihex $< $@
$(BUILD)/%.bin: $(BUILD)/%.elf
	$(CP) -O binary -S $< $@
clean:
	rm -rf $(BUILD)
.PHONY: all clean
