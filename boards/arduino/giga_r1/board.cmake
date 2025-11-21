# SPDX-License-Identifier: Apache-2.0

# keep first
if(CONFIG_XIP AND (CONFIG_STM32_MEMMAP OR CONFIG_BOOTLOADER_MCUBOOT))
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(stm32cubeprogrammer "--extload=AT25SF128A_Arduino_Giga_R1_Loader.stldr")
else()
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
endif()

if(CONFIG_BOARD_ARDUINO_GIGA_R1_STM32H747XX_M7)
board_runner_args(jlink "--device=STM32H747XI_M7" "--speed=4000")
board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd_arduino_giga_r1_m7.cfg")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)
elseif(CONFIG_BOARD_ARDUINO_GIGA_R1_STM32H747XX_M4)
board_runner_args(jlink "--device=STM32H747XI_M4" "--speed=4000")
board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd_arduino_giga_r1_m4.cfg")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu1)
endif()

# keep first
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
