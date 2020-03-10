# Teensy build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-


DIR=teensy
CPU=MIMXRT1062

LD_SCRIPT=lib/$DIR/device/$CPU/MIMXRT1062xxxxx_flexspi_nor.ld

dirs-y += src/$DIR src/generic lib/$DIR/device

CFLAGS += -O3 -mthumb -mcpu=cortex-m3 -Ilib/$DIR/device/$CPU/ -Ilib/CMSIS

CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add source files
src-y += $DIR/main.c $DIR/gpio.c
#src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_timer.c
#src-y += generic/armcm_reset.c generic/crc16_ccitt.c
#src-y += ../lib/$DIR/device/libbbbbbbb.c
#src-$(CONFIG_HAVE_GPIO_ADC) += $DIR/adc.c
#src-$(CONFIG_HAVE_GPIO_I2C) += $DIR/i2c.c
#src-$(CONFIG_HAVE_GPIO_SPI) += $DIR/spi.c
#src-$(CONFIG_USBSERIAL) += $DIR/usbserial.c $DIR/chipid.c
#src-$(CONFIG_USBSERIAL) += generic/usb_cdc.c
#src-$(CONFIG_SERIAL) += $DIR/serial.c generic/serial_irq.c

# Build the additional bin output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Flash rules
flash: $(OUT)klipper.bin
	@echo "  Flashing $< to $(FLASH_DEVICE)"
	$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" $(if $(NOSUDO),--no-sudo) $(OUT)klipper.bin



#SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -D__STARTUP_CLEAR_BSS")
-D__STARTUP_CLEAR_BSS
-DNDEBUG
-D__STARTUP_INITIALIZE_NONCACHEDATA
-mcpu=cortex-m7
-Wall
-mfloat-abi=hard
-mfpu=fpv5-d16
-mthumb
-fno-common
-ffunction-sections
-fdata-sections
-ffreestanding
-fno-builtin
-mapcs
-std=gnu99

#SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")
-DNDEBUG
-DCPU_MIMXRT1062DVL6A
-DSERIAL_PORT_TYPE_UART=1
-Os
-mcpu=cortex-m7
-Wall
-mfloat-abi=hard
-mfpu=fpv5-d16
-mthumb
-MMD
-MP
-fno-common
-ffunction-sections
-fdata-sections
-ffreestanding
-fno-builtin
-mapcs
-std=gnu99

#SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DXIP_EXTERNAL_FLASH=1")
-DXIP_EXTERNAL_FLASH=1
-DXIP_BOOT_HEADER_ENABLE=1
-DNDEBUG
-DCPU_MIMXRT1062DVL6A
-DSERIAL_PORT_TYPE_UART=1
-Os
-mcpu=cortex-m7
-Wall
-mfloat-abi=hard
-mfpu=fpv5-d16
-mthumb
-MMD
-MP
-fno-common
-ffunction-sections
-fdata-sections
-ffreestanding
-fno-builtin
-mapcs
-std=gnu99

#SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -DNDEBUG")
-DNDEBUG
-DSERIAL_PORT_TYPE_UART=1
-Os
-mcpu=cortex-m7
-Wall
-mfloat-abi=hard
-mfpu=fpv5-d16
-mthumb
-MMD
-MP
-fno-common
-ffunction-sections
-fdata-sections
-ffreestanding
-fno-builtin
-mapcs
-fno-rtti
-fno-exceptions

#SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m7")
-mcpu=cortex-m7
-Wall
-mfloat-abi=hard
-mfpu=fpv5-d16
--specs=nano.specs
--specs=nosys.specs
-fno-common
-ffunction-sections
-fdata-sections
-ffreestanding
-fno-builtin
-mthumb
-mapcs
-Xlinker
--gc-sections
-Xlinker
-static
-Xlinker
-z
-Xlinker
#muldefs
-Xlinker
-Map=output.map

#SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mcpu=cortex-m7")
-mcpu=cortex-m7
-Wall
-mfloat-abi=hard
-mfpu=fpv5-d16
--specs=nano.specs
--specs=nosys.specs
-fno-common
-ffunction-sections
-fdata-sections
-ffreestanding
-fno-builtin
-mthumb
-mapcs
-Xlinker
--gc-sections
-Xlinker
-static
-Xlinker
-z
-Xlinker
muldefs
-Xlinker
-Map=output.map

#include_directories(${ProjDirPath}/..)
-I/..
-I/../../../..
-I/../../../../../../CMSIS/Include
-I/../../../../../../devices
-I/../../../../../../devices/MIMXRT1062/drivers
-I/../../../../../../devices/MIMXRT1062
-I/../../../../../../devices/MIMXRT1062/utilities/str
-I/../../../../../../devices/MIMXRT1062/utilities/debug_console
-I/../../../../../../components/uart
-I/../../../../../../components/serial_manager
-I/../../../../../../components/lists
-I/../../../../../../devices/MIMXRT1062/xip
-I/../../../../xip
-I/../../../../../../devices/MIMXRT1062/utilities

add_executable(igpio_led_output.elf 

#"${ProjDirPath}/../gpio_led_output.c"
/../gpio_led_output.c
/../pin_mux.c"
/../pin_mux.h
/../board.c
/../board.h
/../clock_config.c
/../clock_config.h
/../../../../../../devices/MIMXRT1062/drivers/fsl_gpio.c
/../../../../../../devices/MIMXRT1062/drivers/fsl_gpio.h
/../../../../../../devices/MIMXRT1062/drivers/fsl_clock.c
/../../../../../../devices/MIMXRT1062/drivers/fsl_clock.h
/../../../../../../devices/MIMXRT1062/drivers/fsl_common.c
/../../../../../../devices/MIMXRT1062/drivers/fsl_common.h
/../../../../../../devices/MIMXRT1062/MIMXRT1062.h
/../../../../../../devices/MIMXRT1062/MIMXRT1062_features.h
/../../../../../../devices/MIMXRT1062/fsl_device_registers.h
/../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.c
/../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console.h
/../../../../../../devices/MIMXRT1062/utilities/debug_console/fsl_debug_console_conf.h
/../../../../../../devices/MIMXRT1062/utilities/str/fsl_str.c
/../../../../../../devices/MIMXRT1062/utilities/str/fsl_str.h
/../../../../../../components/uart/lpuart_adapter.c
/../../../../../../components/uart/uart.h
/../../../../../../components/serial_manager/serial_manager.c
/../../../../../../components/serial_manager/serial_manager.h
/../../../../../../components/serial_manager/serial_port_internal.h
/../../../../../../components/lists/generic_list.c
/../../../../../../components/lists/generic_list.h
/../../../../../../components/serial_manager/serial_port_uart.c
/../../../../../../components/serial_manager/serial_port_uart.h
/../../../../../../devices/MIMXRT1062/drivers/fsl_lpuart.c
/../../../../../../devices/MIMXRT1062/drivers/fsl_lpuart.h
/../../../../../../devices/MIMXRT1062/gcc/startup_MIMXRT1062.S
/../../../../../../devices/MIMXRT1062/system_MIMXRT1062.c
/../../../../../../devices/MIMXRT1062/system_MIMXRT1062.h
/../../../../../../devices/MIMXRT1062/drivers/fsl_iomuxc.h
/../../../../../../devices/MIMXRT1062/utilities/fsl_assert.c
/../../../../../../devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.c
/../../../../../../devices/MIMXRT1062/xip/fsl_flexspi_nor_boot.h
/../../../../xip/evkmimxrt1060_flexspi_nor_config.c
/../../../../xip/evkmimxrt1060_flexspi_nor_config.h
/../../../../xip/evkmimxrt1060_sdram_ini_dcd.c
/../../../../xip/evkmimxrt1060_sdram_ini_dcd.h
/../../../../../../CMSIS/Include/core_cm7.h
/../../../../../../CMSIS/Include/cmsis_armcc.h
/../../../../../../CMSIS/Include/cmsis_armclang.h
/../../../../../../CMSIS/Include/cmsis_armclang_ltm.h
/../../../../../../CMSIS/Include/cmsis_compiler.h
/../../../../../../CMSIS/Include/cmsis_gcc.h
/../../../../../../CMSIS/Include/cmsis_iccarm.h
/../../../../../../CMSIS/Include/cmsis_version.h
/../../../../../../CMSIS/Include/core_armv81mml.h
/../../../../../../CMSIS/Include/core_armv8mbl.h
/../../../../../../CMSIS/Include/core_armv8mml.h
/../../../../../../CMSIS/Include/mpu_armv7.h
/../../../../../../CMSIS/Include/mpu_armv8.h
/../../../../../../CMSIS/Include/arm_common_tables.h
/../../../../../../CMSIS/Include/arm_const_structs.h
/../../../../../../CMSIS/Include/arm_math.h
/../../../../../../devices/MIMXRT1062/utilities/fsl_sbrk.c


set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/MIMXRT1062xxxxx_ram.ld -static")
set(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -T${ProjDirPath}/MIMXRT1062xxxxx_flexspi_nor.ld -static")

TARGET_LINK_LIBRARIES(igpio_led_output.elf -Wl,--start-group)
#target_link_libraries(igpio_led_output.elf debug m)
#target_link_libraries(igpio_led_output.elf debug c)
#target_link_libraries(igpio_led_output.elf debug gcc)
#target_link_libraries(igpio_led_output.elf debug nosys)
#target_link_libraries(igpio_led_output.elf optimized m)
#target_link_libraries(igpio_led_output.elf optimized c)
#target_link_libraries(igpio_led_output.elf optimized gcc)
#target_link_libraries(igpio_led_output.elf optimized nosys)
#target_link_libraries(igpio_led_output.elf optimized m)
#target_link_libraries(igpio_led_output.elf optimized c)
#target_link_libraries(igpio_led_output.elf optimized gcc)
#target_link_libraries(igpio_led_output.elf optimized nosys)
#target_link_libraries(igpio_led_output.elf optimized m)
#target_link_libraries(igpio_led_output.elf optimized c)
#target_link_libraries(igpio_led_output.elf optimized gcc)
#target_link_libraries(igpio_led_output.elf optimized nosys)
#target_link_libraries(igpio_led_output.elf optimized m)
#target_link_libraries(igpio_led_output.elf optimized c)
#target_link_libraries(igpio_led_output.elf optimized gcc)
#target_link_libraries(igpio_led_output.elf optimized nosys)
#target_link_libraries(igpio_led_output.elf optimized m)
#target_link_libraries(igpio_led_output.elf optimized c)
#target_link_libraries(igpio_led_output.elf optimized gcc)
#target_link_libraries(igpio_led_output.elf optimized nosys)

#TARGET_LINK_LIBRARIES(igpio_led_output.elf -Wl,--end-group)

