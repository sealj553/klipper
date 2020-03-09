# Teensy build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

DIR=teensy

dirs-y += src/$DIR src/generic lib/$DIR/device

CFLAGS += -mthumb -mcpu=cortex-m3 -Ilib/$DIR/device -Ilib/cmsis-core

CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add source files
src-y += $DIR/main.c $DIR/gpio.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_timer.c
src-y += generic/armcm_reset.c generic/crc16_ccitt.c
src-y += ../lib/$DIR/device/system_LPC17xx.c
src-$(CONFIG_HAVE_GPIO_ADC) += $DIR/adc.c
src-$(CONFIG_HAVE_GPIO_I2C) += $DIR/i2c.c
src-$(CONFIG_HAVE_GPIO_SPI) += $DIR/spi.c
src-$(CONFIG_USBSERIAL) += $DIR/usbserial.c $DIR/chipid.c
src-$(CONFIG_USBSERIAL) += generic/usb_cdc.c
src-$(CONFIG_SERIAL) += $DIR/serial.c generic/serial_irq.c

# Build the additional bin output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Flash rules
flash: $(OUT)klipper.bin
	@echo "  Flashing $< to $(FLASH_DEVICE)"
	$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" $(if $(NOSUDO),--no-sudo) $(OUT)klipper.bin
