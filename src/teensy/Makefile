# Teensy build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

DIR=teensy
CPU=MIMXRT1062

LD_SCRIPT=../lib/$DIR/device/$CPU/MIMXRT1062xxxxx_flexspi_nor.ld

#dirs-y += src/$DIR src/generic lib/$DIR/device
#dirs-y += src/$DIR

#CFLAGS += -O3 -mthumb -mcpu=cortex-m3 -Ilib/$DIR/device/$CPU/ -Ilib/CMSIS

#CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
#CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
CFLAGS_klipper.elf += -T $(LD_SCRIPT) -static
#$(OUT)klipper.elf: $(LD_SCRIPT)
$(OUT)klipper.elf: $(LD_SCRIPT)

#set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/MIMXRT1062xxxxx_ram.ld -static")
#set(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -T${ProjDirPath}/MIMXRT1062xxxxx_flexspi_nor.ld -static")

dirs-y += src/$DIR

# Add source files
src-y += $DIR/main.c
#src-y += $DIR/main.c $DIR/gpio.c
#src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_timer.c
#src-y += generic/armcm_reset.c generic/crc16_ccitt.c
#src-y += ../lib/$DIR/device/libbbbbbbb.c
#src-$(CONFIG_HAVE_GPIO_ADC) += $DIR/adc.c
#src-$(CONFIG_HAVE_GPIO_I2C) += $DIR/i2c.c
#src-$(CONFIG_HAVE_GPIO_SPI) += $DIR/spi.c
#src-$(CONFIG_USBSERIAL) += $DIR/usbserial.c $DIR/chipid.c
#src-$(CONFIG_USBSERIAL) += generic/usb_cdc.c
#src-$(CONFIG_SERIAL) += $DIR/serial.c generic/serial_irq.c

#SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DXIP_EXTERNAL_FLASH=1")
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
CFLAGS += -std=gnu99

#SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mcpu=cortex-m7")
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

#include_directories(${ProjDirPath}/..)
#CFLAGS += -I/..
#CFLAGS += -I/../../../..
CFLAGS += -I../lib/teensy/CMSIS/Include
CFLAGS += -I../lib/teensy/devices
CFLAGS += -I../lib/teensy/devices/MIMXRT1062/drivers
CFLAGS += -I../lib/teensy/devices/MIMXRT1062
CFLAGS += -I../lib/teensy/devices/MIMXRT1062/utilities
CFLAGS += -I../lib/teensy/devices/MIMXRT1062/utilities/str
CFLAGS += -I../lib/teensy/devices/MIMXRT1062/utilities/debug_console
CFLAGS += -I../lib/teensy/components/uart
CFLAGS += -I../lib/teensy/components/serial_manager
CFLAGS += -I../lib/teensy/components/lists
CFLAGS += -I../lib/teensy/devices/MIMXRT1062/xip
CFLAGS += -I../lib/teensy/xip
#CFLAGS += -I../lib/teensy/../../devices/MIMXRT1062/utilities

#add_executable(igpio_led_output.elf 

#"${ProjDirPath}/../gpio_led_output.c"

#src-y += ../lib/$DIR/device/libbbbbbbb.c
src-y += /../gpio_led_output.c
src-y += /../pin_mux.c
#src-y += /../pin_mux.h
src-y += /../board.c
#src-y += /../board.h
src-y += /../clock_config.c
#src-y += /../clock_config.h
src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_gpio.c
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_gpio.h
src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_clock.c
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_clock.h
src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_common.c
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_common.h
#src-y += /../../../../../../devices/MIMXRT1062/MIMXRT1062.h
#src-y += /../../../../../../devices/MIMXRT1062/MIMXRT1062_features.h
src-y += /../../../../../../devices/MIMXRT1062/fsl_device_registers.h
src-y += /../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.c
#src-y += /../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.h
#src-y += /../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console_conf.h
src-y += /../../../../../../devices/MIMXRT1062/utilities/str/fsl_str.c
#src-y += /../../../../../../devices/MIMXRT1062/utilities/str/fsl_str.h
src-y += /../../../../../../components/uart/lpuart_adapter.c
#src-y += /../../../../../../components/uart/uart.h
src-y += /../../../../../../components/serial_manager/serial_manager.c
#src-y += /../../../../../../components/serial_manager/serial_manager.h
src-y += /../../../../../../components/serial_manager/serial_port_internal.h
src-y += /../../../../../../components/lists/generic_list.c
#src-y += /../../../../../../components/lists/generic_list.h
#src-y += /../../../../../../components/serial_manager/serial_port_uart.c
src-y += /../../../../../../components/serial_manager/serial_port_uart.h
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_lpuart.c
src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_lpuart.h
#src-y += /../../../../../../devices/MIMXRT1062/gcc/startup_MIMXRT1062.S
src-y += /../../../../../../devices/MIMXRT1062/system_MIMXRT1062.c
#src-y += /../../../../../../devices/MIMXRT1062/system_MIMXRT1062.h
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_iomuxc.h
src-y += /../../../../../../devices/MIMXRT1062/utilities/fsl_assert.c
src-y += /../../../../../../devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.c
#src-y += /../../../../../../devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.h
src-y += /../../../../xip/evkmimxrt1060_flexspi_nor_config.c
#src-y += /../../../../xip/evkmimxrt1060_flexspi_nor_config.h
src-y += /../../../../xip/evkmimxrt1060_sdram_ini_dcd.c
#src-y += /../../../../xip/evkmimxrt1060_sdram_ini_dcd.h
#src-y += /../../../../../../CMSIS/Include/core_cm7.h
#src-y += /../../../../../../CMSIS/Include/cmsis_armcc.h
#src-y += /../../../../../../CMSIS/Include/cmsis_armclang.h
#src-y += /../../../../../../CMSIS/Include/cmsis_armclang_ltm.h
#src-y += /../../../../../../CMSIS/Include/cmsis_compiler.h
#src-y += /../../../../../../CMSIS/Include/cmsis_gcc.h
#src-y += /../../../../../../CMSIS/Include/cmsis_iccarm.h
#src-y += /../../../../../../CMSIS/Include/cmsis_version.h
#src-y += /../../../../../../CMSIS/Include/core_armv81mml.h
#src-y += /../../../../../../CMSIS/Include/core_armv8mbl.h
#src-y += /../../../../../../CMSIS/Include/core_armv8mml.h
#src-y += /../../../../../../CMSIS/Include/mpu_armv7.h
#src-y += /../../../../../../CMSIS/Include/mpu_armv8.h
#src-y += /../../../../../../CMSIS/Include/arm_common_tables.h
#src-y += /../../../../../../CMSIS/Include/arm_const_structs.h
#src-y += /../../../../../../CMSIS/Include/arm_math.h
src-y += /../../../../../../devices/MIMXRT1062/utilities/fsl_sbrk.c


# Build the additional bin output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Flash rules
flash: $(OUT)klipper.bin
	@echo "  Pretending to be Flashing $< to $(FLASH_DEVICE)"
	#$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" $(if $(NOSUDO),--no-sudo) $(OUT)klipper.bin
