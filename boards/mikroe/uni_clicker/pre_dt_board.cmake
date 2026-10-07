# Copyright (c) 2025 Mario Paja
# SPDX-License-Identifier: Apache-2.0

# The UNI Clicker needs an MCU card; require the card variant in the board target.
if(NOT BOARD_QUALIFIERS MATCHES "/")
  message(FATAL_ERROR "No MCU card selected for ${BOARD}/${BOARD_QUALIFIERS}. "
          "Use ${BOARD}/<soc>/<card>, e.g. mikroe_uni_clicker/stm32f429xx/mcu_card_4")
endif()
