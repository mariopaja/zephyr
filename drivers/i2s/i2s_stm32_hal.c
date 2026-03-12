/*
 * Copyright (c) 2026 Mario Paja
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_i2s

#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma/dma_stm32.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2s_stm32);

struct queue_item {
	void *buffer;
	size_t size;
};

struct stream {
	DMA_TypeDef *reg;

	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;

	struct i2s_config i2s_cfg;
	void *mem_block;
	size_t mem_block_len;

	bool master;
	bool last_block;

	int32_t state;
	struct k_msgq queue;

	int (*stream_start)(const struct device *dev, enum i2s_dir dir);
};

struct i2s_stm32_data {
	I2S_HandleTypeDef hi2s;
	DMA_HandleTypeDef hdma_rx;
	DMA_HandleTypeDef hdma_tx;
	struct stream stream_rx;
	struct stream stream_tx;
};

static void dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	ARG_UNUSED(dma_dev);

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
	}
	HAL_DMA_IRQHandler(hdma);
}

static void dma_rx_callback(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	ARG_UNUSED(dma_dev);

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
	}
	HAL_DMA_IRQHandler(hdma);
}

#define I2S_DMA_CHANNEL_INIT(index, dir, src, dest)                                                \
	.stream_##dir = {                                                                          \
		.dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(index, dir)),                              \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                     \
		.reg = (DMA_TypeDef *)DT_REG_ADDR(                                                 \
			DT_PHANDLE_BY_NAME(DT_DRV_INST(index), dmas, dir)),                        \
		.dma_cfg =                                                                         \
			{                                                                          \
				.dma_slot = STM32_DMA_SLOT(index, dir, slot),                      \
				.channel_direction = src##_TO_##dest,                              \
				.dma_callback = dma_##dir##_callback,                              \
				.channel_priority = STM32_DMA_CONFIG_PRIORITY(                     \
					STM32_DMA_CHANNEL_CONFIG(index, dir)),                     \
				.source_data_size = STM32_DMA_CONFIG_##src##_DATA_SIZE(            \
					STM32_DMA_CHANNEL_CONFIG(index, dir)),                     \
				.dest_data_size = STM32_DMA_CONFIG_##dest##_DATA_SIZE(             \
					STM32_DMA_CHANNEL_CONFIG(index, dir)),                     \
			},                                                                         \
		.stream_start = stream_start,                                                      \
	}

#define I2S_STM32_INIT(index)                                                                      \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct stm32_pclken clk_##index[] = STM32_DT_INST_CLOCKS(index);              \
                                                                                                   \
	struct i2s_stm32_data i2s_data_##index = {                                                 \
		.hi2s =                                                                            \
			{                                                                          \
				.Instance = (SPI_TypeDef *)DT_INST_REG_ADDR(index),                \
			},                                                                         \
		IF_ENABLED(DT_INST_DMAS_HAS_NAME(index, rx),			\
			(I2S_DMA_CHANNEL_INIT(index, rx, RX, PERIPHERAL, MEMORY))),          \
			IF_ENABLED(DT_INST_DMAS_HAS_NAME(index, tx),			\
			(I2S_DMA_CHANNEL_INIT(index, tx, TX, MEMORY, PERIPHERAL))),  \
	};                                                                                         \
                                                                                                   \
	K_MSGQ_DEFINE(rx_queue_##index, sizeof(struct queue_item),                                 \
		      CONFIG_I2S_STM32_RX_BLOCK_COUNT, 4);                                         \
	K_MSGQ_DEFINE(tx_queue##index, sizeof(struct queue_item), CONFIG_I2S_STM32_TX_BLOCK_COUNT, \
		      4);                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, NULL, NULL, &i2s_stm32_data_##index, NULL, POST_KERNEL,       \
			      CONFIG_I2S_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(I2S_STM32_INIT)
