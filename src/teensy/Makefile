# Teensy 4.0 build rules

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

src-y += generic/serial_irq.c
src-y += teensy/serial.c

src-y += teensy/adc.c

#src-y += generic/spi.c
#src-y += teensy/spi.c

src-y += teensy/main.c
src-y += teensy/gpio.c
src-y += teensy/pin_mux.c
src-y += teensy/board.c
src-y += teensy/clock_config.c
src-y += teensy/irq.c
src-y += teensy/timer.c

src-y += command.c
src-y += generic/alloc.c
src-y += generic/crc16_ccitt.c
src-y += generic/timer_irq.c

##src-y += generic/armcm_timer.c

#src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_timer.c generic/armcm_reset.c

src-y += ../lib/teensy/devices/MIMXRT1062/system_MIMXRT1062.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_gpio.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_lpuart.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_clock.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_common.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_lpuart_edma.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_dmamux.c
src-y += ../lib/teensy/devices/MIMXRT1062/drivers/fsl_edma.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/str/fsl_str.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/fsl_assert.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/fsl_sbrk.c
src-y += ../lib/teensy/devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.c
src-y += ../lib/teensy/devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.c
src-y += ../lib/teensy/xip/evkmimxrt1060_flexspi_nor_config.c
src-y += ../lib/teensy/xip/evkmimxrt1060_sdram_ini_dcd.c
src-y += ../lib/teensy/components/uart/lpuart_adapter.c
src-y += ../lib/teensy/components/serial_manager/serial_manager.c
src-y += ../lib/teensy/components/serial_manager/serial_port_uart.c
src-y += ../lib/teensy/components/lists/generic_list.c

#src-y += ../lib/teensy/teensyduino/usb_desc.c
#src-y += ../lib/teensy/teensyduino/usb_serial.c
#src-y += ../lib/teensy/teensyduino/usb_serial2.c
#src-y += ../lib/teensy/teensyduino/usb_serial3.c

#dirs-y += src/$BOARD_DIRECTORY src/generic lib/$BOARD_DIRECTORY/device
#dirs-y += src/$BOARD_DIRECTORY

dirs-y += src/
dirs-y += src/teensy/
dirs-y += src/generic/
dirs-y += lib/teensy/
dirs-y += lib/teensy/devices/MIMXRT1062/
dirs-y += lib/teensy/devices/MIMXRT1062/gcc/
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
dirs-y += lib/teensy/teensyduino

CFLAGS = -I$(OUT) -Isrc -I$(OUT)board-generic/ \
    -Wold-style-definition $(call cc-option,$(CC),-Wtype-limits,) \
    -ffunction-sections -fdata-sections
#CFLAGS += -flto -fwhole-program -fno-use-linker-plugin

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

CFLAGS += -Isrc
CFLAGS += -Isrc/teensy
CFLAGS += -Ilib/teensy/CMSIS/Include
CFLAGS += -Ilib/teensy/devices
CFLAGS += -Ilib/teensy/devices/MIMXRT1062
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/gcc
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/drivers
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/utilities
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/utilities/str
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/utilities/debug_console
CFLAGS += -Ilib/teensy/components/uart
CFLAGS += -Ilib/teensy/components/serial_manager
CFLAGS += -Ilib/teensy/components/lists
CFLAGS += -Ilib/teensy/devices/MIMXRT1062/xip
CFLAGS += -Ilib/teensy/xip
CFLAGS += -Ilib/teensy/teensyduino

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

CFLAGS_ASM += -D__STARTUP_CLEAR_BSS
CFLAGS_ASM += -DNDEBUG
CFLAGS_ASM += -D__STARTUP_INITIALIZE_NONCACHEDATA
#CFLAGS_ASM += -D__NO_SYSTEM_INIT
CFLAGS_ASM += -mcpu=cortex-m7
CFLAGS_ASM += -Wall
CFLAGS_ASM += -mfloat-abi=hard
CFLAGS_ASM += -mfpu=fpv5-d16
CFLAGS_ASM += -mthumb
CFLAGS_ASM += -fno-common
CFLAGS_ASM += -ffunction-sections
CFLAGS_ASM += -fdata-sections
CFLAGS_ASM += -ffreestanding
CFLAGS_ASM += -fno-builtin
CFLAGS_ASM += -mapcs
CFLAGS_ASM += -std=gnu11
#CFLAGS_ASM += -Ilib/teensy/devices/MIMXRT1062/gcc/

# Startup routine
ASM_SRC = lib/teensy/devices/MIMXRT1062/gcc/startup_MIMXRT1062.S

$(OUT)$(ASM_SRC).obj: $(ASM_SRC)
	$(Q)$(CC) $(CFLAGS_ASM) $(ASM_SRC) -c -o $(OUT)$(ASM_SRC).obj

OBJS_klipper.elf += $(OUT)$(ASM_SRC).obj

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

# Linker script
LD_SCRIPT=lib/teensy/devices/MIMXRT1062/gcc/MIMXRT1062xxxxx_flexspi_nor.ld

CFLAGS_klipper.elf += -T $(LD_SCRIPT) -static
$(OUT)klipper.elf: $(LD_SCRIPT)

# Build the additional hex output file
target-y += $(OUT)klipper.hex

$(OUT)klipper.hex: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O ihex $< $@

# Flash rules
flash: $(OUT)klipper.hex
	@echo "Flashing $< to $(FLASH_DEVICE)"
	teensy_loader_cli -v -w --mcu=imxrt1062 $(OUT)klipper.hex
