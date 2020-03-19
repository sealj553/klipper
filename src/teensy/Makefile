# Teensy 4.0 build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

#DIR=$(BOARD_DIRECTORY)

LD_SCRIPT=lib/teensy/devices/MIMXRT1062/gcc/MIMXRT1062xxxxx_flexspi_nor.ld

#dirs-y += src/$BOARD_DIRECTORY src/generic lib/$BOARD_DIRECTORY/device
#dirs-y += src/$BOARD_DIRECTORY

#CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
CFLAGS_klipper.elf += -T $(LD_SCRIPT) -static
$(OUT)klipper.elf: $(LD_SCRIPT)

dirs-y += src/teensy/
dirs-y += lib/teensy/
dirs-y += lib/teensy/devices/MIMXRT1062/
dirs-y += lib/teensy/devices/MIMXRT1062/drivers/
dirs-y += lib/teensy/devices/MIMXRT1062/utilities/
dirs-y += lib/teensy/devices/MIMXRT1062/utilities/str
dirs-y += lib/teensy/devices/MIMXRT1062/utilities/debug_console
dirs-y += lib/teensy/devices/MIMXRT1062/xip/
dirs-y += lib/teensy/xip/
dirs-y += lib/teensy/components
dirs-y += lib/teensy/components/button
dirs-y += lib/teensy/components/codec
dirs-y += lib/teensy/components/common_task
dirs-y += lib/teensy/components/crc
dirs-y += lib/teensy/components/flash
dirs-y += lib/teensy/components/ft5406_rt
dirs-y += lib/teensy/components/fxos8700cq
dirs-y += lib/teensy/components/gpio
dirs-y += lib/teensy/components/i2c
dirs-y += lib/teensy/components/led
dirs-y += lib/teensy/components/lists
dirs-y += lib/teensy/components/mem_manager
dirs-y += lib/teensy/components/osa
dirs-y += lib/teensy/components/panic
dirs-y += lib/teensy/components/phyksz8081
dirs-y += lib/teensy/components/rng
dirs-y += lib/teensy/components/serial_manager
dirs-y += lib/teensy/components/spi
dirs-y += lib/teensy/components/timer
dirs-y += lib/teensy/components/timer_manager
dirs-y += lib/teensy/components/uart
dirs-y += lib/teensy/components/video

# Add source files
#src-y += $(BOARD_DIRECTORY)/main.c
#src-y += $BOARD_DIRECTORY/main.c $BOARD_DIRECTORY/gpio.c
#src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_timer.c
#src-y += generic/armcm_reset.c generic/crc16_ccitt.c
#src-y += ../lib/$BOARD_DIRECTORY/device/libbbbbbbb.c
#src-$(CONFIG_HAVE_GPIO_ADC) += $BOARD_DIRECTORY/adc.c
#src-$(CONFIG_HAVE_GPIO_I2C) += $BOARD_DIRECTORY/i2c.c
#src-$(CONFIG_HAVE_GPIO_SPI) += $BOARD_DIRECTORY/spi.c
#src-$(CONFIG_USBSERIAL) += $BOARD_DIRECTORY/usbserial.c $BOARD_DIRECTORY/chipid.c
#src-$(CONFIG_USBSERIAL) += generic/usb_cdc.c
#src-$(CONFIG_SERIAL) += $BOARD_DIRECTORY/serial.c generic/serial_irq.c

CFLAGS += -DXIP_EXTERNAL_FLASH=1
CFLAGS += -DXIP_BOOT_HEADER_ENABLE=1
CFLAGS += -DNDEBUG
CFLAGS += -DCPU_MIMXRT1062DVL6A
CFLAGS += -DSERIAL_PORT_TYPE_UART=1
CFLAGS += -O3
CFLAGS += -mcpu=cortex-m7
CFLAGS += -Wall
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv5-d16
CFLAGS += -mthumb
CFLAGS += -MMD
CFLAGS += -MP
CFLAGS += -fno-common
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -ffreestanding
CFLAGS += -fno-builtin
CFLAGS += -mapcs
CFLAGS += -std=gnu11

CFLAGS_klipper.elf += -mcpu=cortex-m7
CFLAGS_klipper.elf += -Wall
CFLAGS_klipper.elf += -mfloat-abi=hard
CFLAGS_klipper.elf += -mfpu=fpv5-d16
CFLAGS_klipper.elf += --specs=nano.specs
CFLAGS_klipper.elf += --specs=nosys.specs
CFLAGS_klipper.elf += -fno-common
CFLAGS_klipper.elf += -ffunction-sections
CFLAGS_klipper.elf += -fdata-sections
CFLAGS_klipper.elf += -ffreestanding
CFLAGS_klipper.elf += -fno-builtin
CFLAGS_klipper.elf += -mthumb
CFLAGS_klipper.elf += -mapcs
CFLAGS_klipper.elf += -Xlinker
CFLAGS_klipper.elf += --gc-sections
CFLAGS_klipper.elf += -Xlinker
CFLAGS_klipper.elf += -static
CFLAGS_klipper.elf += -Xlinker
CFLAGS_klipper.elf += -z
CFLAGS_klipper.elf += -Xlinker
CFLAGS_klipper.elf += muldefs
CFLAGS_klipper.elf += -Xlinker
CFLAGS_klipper.elf += -Map=output.map

CFLAGS += -Ilib/teensy/CMSIS/Include
CFLAGS += -Ilib/teensy/devices
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/drivers
CFLAGS += -Ilib/teensy/devices/MIMXRT1062
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/utilities
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/utilities/str
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/utilities/debug_console
CFLAGS += -Ilib/teensy/components/uart
CFLAGS += -Ilib/teensy/components/serial_manager
CFLAGS += -Ilib/teensy/components/lists
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/xip
CFLAGS += -Ilib/teensy/xip
CFLAGS += -Isrc/teensy

src-y += teensy/pin_mux.c
src-y += teensy/board.c
src-y += teensy/clock_config.c
src-y += ../lib/teensy/devices/MIMXRT1062/system_MIMXRT1062.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_gpio.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_clock.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_common.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/str/fsl_str.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/fsl_assert.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/fsl_sbrk.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.c
src-y += ../lib/teensy/devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.c
src-y += ../lib/teensy/xip/evkmimxrt1060_flexspi_nor_config.c
src-y += ../lib/teensy/xip/evkmimxrt1060_sdram_ini_dcd.c
src-y += ../lib/teensy/components/uart/lpuart_adapter.c
src-y += ../lib/teensy/components/serial_manager/serial_manager.c
src-y += ../lib/teensy/components/lists/generic_list.c

# Build the additional bin output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O ihex $< $@

# Flash rules
flash: $(OUT)klipper.bin
	@echo "Flashing $< to $(FLASH_DEVICE)"
	teensy_loader_cli --mcu=TEENSY40 -w $(OUT)klipper.bin
