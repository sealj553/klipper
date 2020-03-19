# Teensy build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

#DIR=$(BOARD_DIRECTORY)

LD_SCRIPT=lib/teensy/devices/MIMXRT1062/gcc/MIMXRT1062xxxxx_flexspi_nor.ld

#dirs-y += src/$BOARD_DIRECTORY src/generic lib/$BOARD_DIRECTORY/device
#dirs-y += src/$BOARD_DIRECTORY

#CFLAGS += -O3 -mthumb -mcpu=cortex-m3 -Ilib/$BOARD_DIRECTORY/device/$CPU/ -Ilib/CMSIS

#CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
#CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
CFLAGS_klipper.elf += -T $(LD_SCRIPT) -static
$(OUT)klipper.elf: $(LD_SCRIPT)

#set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/MIMXRT1062xxxxx_ram.ld -static")
#set(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -T${ProjDirPath}/MIMXRT1062xxxxx_flexspi_nor.ld -static")

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




#dirs-y += lib/teensy/devices/MIMXRT1062/project_template

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
#CFLAGS += -MMD
#CFLAGS += -MP
CFLAGS += -fno-common
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -ffreestanding
CFLAGS += -fno-builtin
CFLAGS += -mapcs
CFLAGS += -std=gnu11

###CFLAGS += -Ilib/teensy/devices/MIMXRT1062/


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

#CFLAGS += -I/..
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
	$(Q)$(OBJCOPY) -O binary $< $@

# Flash rules
flash: $(OUT)klipper.bin
	@echo "  Pretending to be Flashing $< to $(FLASH_DEVICE)"
	#$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" $(if $(NOSUDO),--no-sudo) $(OUT)klipper.bin



#src-y += devices/MIMXRT1062/drivers/fsl_gpio.c
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_gpio.h
##src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_clock.c
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_clock.h
##src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_common.c
#src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_common.h
#src-y += /../../../../../../devices/MIMXRT1062/MIMXRT1062.h
#src-y += /../../../../../../devices/MIMXRT1062/MIMXRT1062_features.h
#src-y += /../../../../../../devices/MIMXRT1062/fsl_device_registers.h
##src-y += /../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.c
#src-y += /../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.h
#src-y += /../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console_conf.h
###src-y += /../../../../../../devices/MIMXRT1062/utilities/str/fsl_str.c
####src-y += /../../../../../../devices/MIMXRT1062/utilities/str/fsl_str.h
###src-y += /../../../../../../components/uart/lpuart_adapter.c
####src-y += /../../../../../../components/uart/uart.h
###src-y += /../../../../../../components/serial_manager/serial_manager.c
####src-y += /../../../../../../components/serial_manager/serial_manager.h
###src-y += /../../../../../../components/serial_manager/serial_port_internal.h
###src-y += /../../../../../../components/lists/generic_list.c
####src-y += /../../../../../../components/lists/generic_list.h
####src-y += /../../../../../../components/serial_manager/serial_port_uart.c
###src-y += /../../../../../../components/serial_manager/serial_port_uart.h
####src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_lpuart.c
###src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_lpuart.h
####src-y += /../../../../../../devices/MIMXRT1062/gcc/startup_MIMXRT1062.S
###src-y += /../../../../../../devices/MIMXRT1062/system_MIMXRT1062.c
####src-y += /../../../../../../devices/MIMXRT1062/system_MIMXRT1062.h
####src-y += /../../../../../../devices/MIMXRT1062/drivers/fsl_iomuxc.h
###src-y += /../../../../../../devices/MIMXRT1062/utilities/fsl_assert.c
###src-y += /../../../../../../devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.c
####src-y += /../../../../../../devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.h
###src-y += /../../../../xip/evkmimxrt1060_flexspi_nor_config.c
####src-y += /../../../../xip/evkmimxrt1060_flexspi_nor_config.h
###src-y += /../../../../xip/evkmimxrt1060_sdram_ini_dcd.c
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
#src-y += /../../../../../../devices/MIMXRT1062/utilities/fsl_sbrk.c


