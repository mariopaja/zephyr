/*
 * Copyright (c) 2024-2025 ZAL Zentrum für Angewandte Luftfahrtforschung GmbH
 * Copyright (c) 2024-2026 Mario Paja
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_sai

#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/cache.h>

#include <stm32_ll_dma.h>
#include <stm32_bitops.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2s_stm32_sai, CONFIG_I2S_LOG_LEVEL);

#if defined(CONFIG_I2S_STM32_SAI_PDM)
#include <OpenPDMFilter.h>
#endif

enum mclk_divider {
	MCLK_NO_DIV,
	MCLK_DIV_256,
	MCLK_DIV_512
};

enum sai_mode {
	SAI_MODE_I2S,
	SAI_MODE_PDM,
};

static const uint32_t dma_priority[] = {
#if defined(CONFIG_DMA_STM32U5)
	DMA_LOW_PRIORITY_LOW_WEIGHT,
	DMA_LOW_PRIORITY_MID_WEIGHT,
	DMA_LOW_PRIORITY_HIGH_WEIGHT,
	DMA_HIGH_PRIORITY,
#else
	DMA_PRIORITY_LOW,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VERY_HIGH,
#endif
};

#if defined(CONFIG_DMA_STM32U5)
static const uint32_t dma_src_size[] = {
	DMA_SRC_DATAWIDTH_BYTE,
	DMA_SRC_DATAWIDTH_HALFWORD,
	DMA_SRC_DATAWIDTH_WORD,
};
static const uint32_t dma_dest_size[] = {
	DMA_DEST_DATAWIDTH_BYTE,
	DMA_DEST_DATAWIDTH_HALFWORD,
	DMA_DEST_DATAWIDTH_WORD,
};
#else
static const uint32_t dma_p_size[] = {
	DMA_PDATAALIGN_BYTE,
	DMA_PDATAALIGN_HALFWORD,
	DMA_PDATAALIGN_WORD,
};
static const uint32_t dma_m_size[] = {
	DMA_MDATAALIGN_BYTE,
	DMA_MDATAALIGN_HALFWORD,
	DMA_MDATAALIGN_WORD,
};
#endif

static const uint32_t sai_fifo_threshold[] = {
	SAI_FIFOTHRESHOLD_EMPTY,
	SAI_FIFOTHRESHOLD_1QF,
	SAI_FIFOTHRESHOLD_HF,
	SAI_FIFOTHRESHOLD_3QF,
	SAI_FIFOTHRESHOLD_FULL,
};

static const uint32_t sai_pdm_clock_enable[] = {
	SAI_PDM_CLOCK1_ENABLE,
	SAI_PDM_CLOCK2_ENABLE,
	SAI_PDM_CLOCK1_ENABLE | SAI_PDM_CLOCK2_ENABLE,
};

/*
 * PDM frame/slot layout per number of microphone pairs. Init.PdmInit only
 * captures the raw PDM bitstream into the FIFO 8 bits at a time (DataSize =
 * SAI_DATASIZE_8) -- it does not demodulate to PCM in hardware. Each mic
 * pair takes 2 slots (one per channel of the pair), confirmed against both
 * the ST BSP's MX_SAI1_Init() (SlotNumber=2 for MicPairsNbr=1) and the
 * STM32CubeU3 SAI_AudioRecord_PDM example (FrameLength=16, SlotNumber=2,
 * DataSize=8 for 1 mic pair). SlotSize follows Init.DataSize
 * (SAI_SLOTSIZE_DATASIZE) rather than forcing a fixed slot width.
 */
#if defined(CONFIG_I2S_STM32_SAI_PDM)
struct sai_pdm_frame_cfg {
	uint32_t frame_length;
	uint32_t slot_number;
};

static const struct sai_pdm_frame_cfg sai_pdm_frame_cfg[] = {
	{.frame_length = 16, .slot_number = 2}, /* 1 pair: 2 slots x 8-bit DataSize */
	{.frame_length = 32, .slot_number = 4}, /* 2 pairs */
	{.frame_length = 48, .slot_number = 6}, /* 3 pairs */
};
#endif

struct queue_item {
	void *buffer;
	size_t size;
};

struct stream {
	DMA_TypeDef *reg;

	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;

	/* DMA size of a source block transfer in bytes according SAI data size. */
	uint8_t dma_src_size;

	struct i2s_config i2s_cfg;
	void *mem_block;
	size_t mem_block_len;

	bool master;
	bool last_block;

	int32_t state;
	struct k_msgq queue;

	int (*stream_start)(const struct device *dev, enum i2s_dir dir);
	void (*queue_drop)(const struct device *dev);
};

#if defined(CONFIG_I2S_STM32_SAI_PDM)
typedef void (*sai_pdm_filter_fn_t)(uint8_t *data, uint16_t *data_out, uint16_t volume,
				     TPDMFilter_InitStruct *param);

/*
 * PDM->PCM decimation state (ST's OpenPDMFilter, one mic pair / stereo
 * only for now). Open_PDM_Filter_64()/_128() each process one "group":
 * Fs/1000 output samples at once, consuming 8 (or 16, for _128) raw bytes
 * per channel per sample. group_raw_bytes/group_pcm_samples are that
 * group's sizes, precomputed once in sai_sub_pdm_conf().
 */
struct sai_pdm_data {
	TPDMFilter_InitStruct filter[2]; /* [0] = left, [1] = right */
	sai_pdm_filter_fn_t filter_fn;
	uint32_t group_raw_bytes;
	uint32_t group_pcm_samples; /* per channel, per group */
	uint16_t volume;
	bool mono;
	uint8_t mono_channel; /* which of filter[]/the raw byte pair mono uses */

	/*
	 * Second single-pole lowpass stage, cascaded after the filter's own
	 * LP_HZ smoothing (see sai_sub_pdm_conf()) for an effective 12dB/
	 * octave rolloff instead of 6dB/octave -- cuts more noise-shaped PDM
	 * quantization noise without pushing LP_HZ itself any lower (which
	 * trades away treble). smooth_state persists across calls, one per
	 * channel, same as the library's own OldZ/OldOut.
	 */
	uint16_t smooth_alfa;
	int32_t smooth_state[2];
};
#endif

struct stm32_sai_sub_data {
	SAI_HandleTypeDef hsai;
	DMA_HandleTypeDef hdma;
	struct stream stream;
#if defined(CONFIG_I2S_STM32_SAI_PDM)
	struct sai_pdm_data pdm;
	bool pdm_active;
#endif
};

struct stm32_sai_sub_cfg {
	const struct pinctrl_dev_config *pincfg;
	bool mclk_enable;
	enum mclk_divider mclk_div;
	bool synchronous;
	enum i2s_dir dir;
	enum sai_mode mode;
	uint8_t pdm_mic_pairs;
	uint32_t pdm_clock_enable;
	uint8_t pdm_volume;
	bool pdm_mono;
	uint8_t pdm_mono_channel;

	const struct device *controller;
};

struct stm32_sai_cfg {
	const struct stm32_pclken sai_ck;
	const struct stm32_pclken sai_ker_ck;
	const struct stm32_pclken sai_b_ker_ck; /* Dedicated to SAIn_B, if applicable */
	bool has_sai_ker_ck: 1;
	bool has_sai_b_ker_ck: 1;
};

static inline void sai_sub_disable(SAI_HandleTypeDef *hsai, i2s_opt_t options)
{
	if ((options & I2S_OPT_BIT_CLK_GATED) == 0) {
		LOG_DBG("SAI sub-block %p not disabled: bit clock gating disabled", hsai->Instance);
		return;
	}

	if (hsai->Init.Synchro == SAI_SYNCHRONOUS) {
		LOG_DBG("SAI sub-block %p not disabled: configured as synchronous peripheral",
			hsai->Instance);
		return;
	}

	__HAL_SAI_DISABLE(hsai);

	LOG_DBG("SAI Disabled");
}

#if defined(CONFIG_I2S_STM32_SAI_PDM)
/*
 * Fixed reference gain the filter is calibrated against (Init.MaxVolume
 * below); NOT the user-facing gain knob. See sai_pdm_decimate().
 */
#define SAI_PDM_MAX_VOLUME 64U

/*
 * Decimate a raw PDM capture block to PCM in place. Safe to overwrite buf
 * as it goes: each output group is always smaller than, and fully behind,
 * the raw group it was derived from.
 *
 * Channel A always goes through the filter at its native interleaved
 * stride (matching In_MicChannels=2, which the raw byte layout requires
 * regardless of how many mics are actually wired). In mono mode channel B
 * is skipped entirely and the channel-A samples -- still strided every
 * other int16_t at that point -- are compacted down to a contiguous mono
 * buffer afterward, rather than handed to the app interleaved with
 * channel B's unconnected-mic noise.
 */
static size_t sai_pdm_decimate(struct stm32_sai_sub_data *sub_data, void *buf, size_t raw_len)
{
	struct sai_pdm_data *pdm = &sub_data->pdm;
	uint8_t *raw = buf;
	int16_t *pcm = buf;
	uint32_t n_groups = raw_len / pdm->group_raw_bytes;
	uint32_t out_index = 0;

	uint8_t ch_a = pdm->mono ? pdm->mono_channel : 0U;

	/*
	 * Always decode at the library's calibrated reference gain
	 * (MaxVolume) for full output resolution -- Open_PDM_Filter_64's
	 * runtime `volume` argument shares one div_const computed once from
	 * MaxVolume at init, so passing a low volume here doesn't cleanly
	 * attenuate: it throws away output bits (e.g. volume=4 of
	 * MaxVolume=64 only ever reaches 1/16 of the output range, ~4 bits
	 * of resolution lost) rather than scaling the full-resolution
	 * signal down. Apply pdm->volume afterward instead, at int32
	 * precision. (Zephyr's own dmic_mpxxdtyy.c driver, using this same
	 * library, follows the same pattern: it always passes MaxVolume
	 * itself as the runtime gain argument.)
	 */
	for (uint32_t i = 0; i < n_groups; i++) {
		uint8_t *group = raw + i * pdm->group_raw_bytes;

		pdm->filter_fn(group + ch_a, (uint16_t *)&pcm[out_index], SAI_PDM_MAX_VOLUME,
				&pdm->filter[ch_a]);
		if (!pdm->mono) {
			pdm->filter_fn(group + 1, (uint16_t *)&pcm[out_index + 1],
					SAI_PDM_MAX_VOLUME, &pdm->filter[1]);
		}
		out_index += pdm->group_pcm_samples * 2U;
	}

	if (pdm->mono) {
		uint32_t n_samples = out_index / 2U;

		for (uint32_t i = 0; i < n_samples; i++) {
			int16_t sample = (int16_t)(((int32_t)pcm[i * 2U] * pdm->volume) /
						    SAI_PDM_MAX_VOLUME);

			pdm->smooth_state[0] = ((256 - pdm->smooth_alfa) * pdm->smooth_state[0] +
						 pdm->smooth_alfa * (int32_t)sample) >>
						8;
			pcm[i] = (int16_t)pdm->smooth_state[0];
		}
		return n_samples * sizeof(int16_t);
	}

	for (uint32_t i = 0; i < out_index; i++) {
		int16_t sample = (int16_t)(((int32_t)pcm[i] * pdm->volume) / SAI_PDM_MAX_VOLUME);
		int32_t *state = &pdm->smooth_state[i % 2U];

		*state = ((256 - pdm->smooth_alfa) * (*state) + pdm->smooth_alfa * (int32_t)sample) >>
			 8;
		pcm[i] = (int16_t)*state;
	}

	return out_index * sizeof(int16_t);
}
#endif

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	struct stm32_sai_sub_data *sub_data = CONTAINER_OF(hsai, struct stm32_sai_sub_data, hsai);
	struct stream *stream = &sub_data->stream;
	int ret;

	/* Exit the callback, Stream is stopped */
	if (stream->state == I2S_STATE_ERROR) {
		goto exit;
	}

	if (stream->mem_block == NULL) {
		if (stream->state != I2S_STATE_READY) {
			stream->state = I2S_STATE_ERROR;
			LOG_ERR("RX mem_block NULL");
			sai_sub_disable(hsai, stream->i2s_cfg.options);
			goto exit;
		} else {
			return;
		}
	}

#if defined(CONFIG_I2S_STM32_SAI_PDM)
	if (sub_data->pdm_active) {
		/*
		 * Temporary capture/decimation debug prints -- raw bytes are
		 * only visible here, before decimation overwrites them
		 * in-place. Re-disable (comment back out) once the raw/PCM
		 * split doesn't need re-checking anymore; steady-state sample
		 * printing belongs in the application, not here.
		 */
		static uint32_t dbg_count;
		uint8_t *raw = stream->mem_block;
		bool dbg_print = (dbg_count++ % 50U) == 0U;

		if (dbg_print) {
			LOG_DBG("PDM raw: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x"
				" %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],
				raw[8], raw[9], raw[10], raw[11], raw[12], raw[13], raw[14],
				raw[15], raw[16], raw[17], raw[18], raw[19]);
		}

		stream->mem_block_len =
			sai_pdm_decimate(sub_data, stream->mem_block, stream->mem_block_len);

		if (dbg_print) {
			int16_t *pcm = stream->mem_block;

			LOG_DBG("PDM pcm: %d %d %d %d %d %d %d %d %d %d", pcm[0], pcm[1],
				pcm[2], pcm[3], pcm[4], pcm[5], pcm[6], pcm[7], pcm[8], pcm[9]);
		}
	}
#endif

	struct queue_item item = {.buffer = stream->mem_block, .size = stream->mem_block_len};

	ret = k_msgq_put(&stream->queue, &item, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		LOG_DBG("Stopping RX ...");
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	ret = k_mem_slab_alloc(stream->i2s_cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	stream->mem_block_len = stream->i2s_cfg.block_size;

	if (HAL_SAI_Receive_DMA(hsai, stream->mem_block,
				stream->mem_block_len / stream->dma_src_size) != HAL_OK) {
		LOG_ERR("HAL_SAI_Receive_DMA: <FAILED>");
	}

exit:
	/* EXIT */
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	struct stm32_sai_sub_data *sub_data = CONTAINER_OF(hsai, struct stm32_sai_sub_data, hsai);
	struct stream *stream = &sub_data->stream;
	void *mem_block_tmp = stream->mem_block;
	struct queue_item item;
	int ret;

	if (stream->state == I2S_STATE_ERROR) {
		LOG_ERR("TX bad status: %d, Stopping...", stream->state);
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	if (stream->mem_block == NULL) {
		if (stream->state != I2S_STATE_READY) {
			stream->state = I2S_STATE_ERROR;
			LOG_ERR("TX mem_block NULL");
			sai_sub_disable(hsai, stream->i2s_cfg.options);
			goto exit;
		} else {
			return;
		}
	}

	if (stream->last_block) {
		LOG_DBG("TX Stopped ...");
		stream->state = I2S_STATE_READY;
		stream->mem_block = NULL;
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	/* Exit callback, no more data in the queue */
	/* Reset I2S state */
	if (k_msgq_num_used_get(&stream->queue) == 0) {
		LOG_DBG("Exit TX callback, no more data in the queue");
		stream->state = I2S_STATE_READY;
		stream->mem_block = NULL;
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	ret = k_msgq_get(&stream->queue, &item, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		sai_sub_disable(hsai, stream->i2s_cfg.options);
		goto exit;
	}

	stream->mem_block = item.buffer;
	stream->mem_block_len = item.size;

	sys_cache_data_flush_range(stream->mem_block, stream->mem_block_len);

	if (HAL_SAI_Transmit_DMA(hsai, stream->mem_block,
				 stream->mem_block_len / stream->dma_src_size) != HAL_OK) {
		LOG_ERR("HAL_SAI_Transmit_DMA: <FAILED>");
	}

exit:
	/* Free memory slab & exit */
	k_mem_slab_free(stream->i2s_cfg.mem_slab, mem_block_tmp);
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	switch (HAL_SAI_GetError(hsai)) {
	case HAL_SAI_ERROR_NONE:
		LOG_INF("No error");
		break;
	case HAL_SAI_ERROR_OVR:
		LOG_WRN("Overrun Error");
		break;
	case HAL_SAI_ERROR_UDR:
		LOG_WRN("Underrun Error");
		break;
	case HAL_SAI_ERROR_AFSDET:
		LOG_WRN("Anticipated Frame synchronisation detection");
		break;
	case HAL_SAI_ERROR_LFSDET:
		LOG_WRN("Late Frame synchronisation detection");
		break;
	case HAL_SAI_ERROR_CNREADY:
		LOG_WRN("codec not ready");
		break;
	case HAL_SAI_ERROR_TIMEOUT:
		LOG_WRN("Timeout error");
		break;
	case HAL_SAI_ERROR_DMA:
		LOG_WRN("DMA error");
		break;
	default:
		LOG_ERR("Unknown error");
	}
}

static int stm32_sai_clock_en(const struct device *dev)
{
	const struct stm32_sai_cfg *sai_cfg = dev->config;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int ret;

	/* Turn on SAI peripheral clock */
	ret = clock_control_on(clk, (clock_control_subsys_t)&sai_cfg->sai_ck);
	if (ret != 0) {
		return -EIO;
	}

	/* Configure shared SAIn_A & SAIn_B or dedicated SAIn_A kernel clock */
	if (sai_cfg->has_sai_ker_ck) {
		ret = clock_control_configure(clk, (clock_control_subsys_t)&sai_cfg->sai_ker_ck,
					      NULL);
		if (ret != 0) {
			return -EIO;
		}
	}

	/* Configure dedicated SAI B kernel clock */
	if (sai_cfg->has_sai_b_ker_ck) {
		ret = clock_control_configure(clk, (clock_control_subsys_t)&sai_cfg->sai_b_ker_ck,
					      NULL);
		if (ret != 0) {
			return -EIO;
		}
	}

	return 0;
}

static int sai_sub_dma_init(const struct device *dev)
{
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	struct dma_config *dma_cfg = &sub_data->stream.dma_cfg;
	int ret;

	SAI_HandleTypeDef *hsai = &sub_data->hsai;
	DMA_HandleTypeDef *hdma = &sub_data->hdma;

	if (!device_is_ready(stream->dma_dev)) {
		LOG_ERR("%s DMA device not ready", stream->dma_dev->name);
		return -ENODEV;
	}

	/* Proceed to the minimum Zephyr DMA driver init */
	dma_cfg->user_data = hdma;

	/* HACK: This field is used to inform driver that it is overridden */
	dma_cfg->linked_channel = STM32_DMA_HAL_OVERRIDE;

	ret = dma_config(stream->dma_dev, stream->dma_channel, dma_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel %d", stream->dma_channel);
		return ret;
	}

	hdma->Instance = STM32_DMA_GET_INSTANCE(stream->reg, stream->dma_channel);
	hdma->Init.Mode = DMA_NORMAL;

	if (dma_cfg->channel_priority >= ARRAY_SIZE(dma_priority)) {
		LOG_ERR("Invalid DMA channel priority");
		return -EINVAL;
	}
	hdma->Init.Priority = dma_priority[dma_cfg->channel_priority];

#if defined(DMA_CHANNEL_1)
	hdma->Init.Channel = dma_cfg->dma_slot * DMA_CHANNEL_1;
#else
	hdma->Init.Request = dma_cfg->dma_slot;
#endif

	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		LOG_ERR("Source and destination data sizes are not aligned");
		return -EINVAL;
	}

	int idx = find_lsb_set(dma_cfg->source_data_size) - 1;

#if defined(CONFIG_DMA_STM32U5)
	if (idx >= ARRAY_SIZE(dma_src_size)) {
		LOG_ERR("Invalid source and destination DMA data size");
		return -EINVAL;
	}

	hdma->Init.SrcDataWidth = dma_src_size[idx];
	hdma->Init.DestDataWidth = dma_dest_size[idx];
	hdma->Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
	hdma->Init.SrcBurstLength = 1;
	hdma->Init.DestBurstLength = 1;
	hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
	hdma->Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
#else
	if (idx >= ARRAY_SIZE(dma_m_size)) {
		LOG_ERR("Invalid peripheral and memory DMA data size");
		return -EINVAL;
	}

	hdma->Init.PeriphDataAlignment = dma_p_size[idx];
	hdma->Init.MemDataAlignment = dma_m_size[idx];
	hdma->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma->Init.MemInc = DMA_MINC_ENABLE;
#endif

#if defined(DMA_FIFOMODE_DISABLE)
	hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif

	if (dma_cfg->channel_direction == (enum dma_channel_direction)MEMORY_TO_PERIPHERAL) {
		hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;

#if defined(CONFIG_DMA_STM32U5)
		hdma->Init.SrcInc = DMA_SINC_INCREMENTED;
		hdma->Init.DestInc = DMA_DINC_FIXED;
#endif

		__HAL_LINKDMA(hsai, hdmatx, sub_data->hdma);
	} else {
		hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;

#if defined(CONFIG_DMA_STM32U5)
		hdma->Init.SrcInc = DMA_SINC_FIXED;
		hdma->Init.DestInc = DMA_DINC_INCREMENTED;
#endif

		__HAL_LINKDMA(hsai, hdmarx, sub_data->hdma);
	}

	if (HAL_DMA_Init(&sub_data->hdma) != HAL_OK) {
		LOG_ERR("HAL_DMA_Init: <FAILED>");
		return -EIO;
	}

#if defined(CONFIG_SOC_SERIES_STM32N6X)
	if (HAL_DMA_ConfigChannelAttributes(&sub_data->hdma, DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
					    DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK) {
		LOG_ERR("HAL_DMA_ConfigChannelAttributes: <Failed>");
		return -EIO;
	}
#elif defined(CONFIG_DMA_STM32U5)
	if (HAL_DMA_ConfigChannelAttributes(&sub_data->hdma, DMA_CHANNEL_NPRIV) != HAL_OK) {
		LOG_ERR("HAL_DMA_ConfigChannelAttributes: <Failed>");
		return -EIO;
	}
#endif

	LOG_DBG("dma@%08x Init: <OK>", (uint32_t)hdma->Instance);

	return 0;
}

static int sai_sub_init(const struct device *dev)
{
	struct stm32_sai_sub_data *sub_data = dev->data;
	const struct stm32_sai_sub_cfg *sub_cfg = dev->config;
	struct stream *stream = &sub_data->stream;
	int ret = 0;

	/* Configure DT provided pins */
	ret = pinctrl_apply_state(sub_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("SAI Sub-Block pinctrl setup: <FAILED>, ret: %d", ret);
		return ret;
	}

	if (!device_is_ready(sub_data->stream.dma_dev)) {
		LOG_ERR("%s device not ready", sub_data->stream.dma_dev->name);
		return -ENODEV;
	}

	ret = k_msgq_alloc_init(&sub_data->stream.queue, sizeof(struct queue_item),
				CONFIG_I2S_STM32_SAI_BLOCK_COUNT);
	if (ret < 0) {
		LOG_ERR("k_msgq_alloc_init(): <FAILED>, ret: %d", ret);
		return ret;
	}

	/* Initialize DMA */
	ret = sai_sub_dma_init(dev);
	if (ret < 0) {
		LOG_ERR("SAI Sub-Block DMA Init <FAILED>, ret: %d", ret);
		return ret;
	}

	/* State set to not ready until successfully configured */
	stream->state = I2S_STATE_NOT_READY;

	return 0;
}

static void dma_callback(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	ARG_UNUSED(dma_dev);

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
	}
	HAL_DMA_IRQHandler(hdma);
}

#if defined(CONFIG_SOC_SERIES_STM32F4X)
static int stm32_sai_sub_f4_clk_src_conf(const struct device *dev)
{
	const struct stm32_sai_sub_cfg *sub_cfg = dev->config;
	const struct stm32_sai_cfg *sai_cfg = sub_cfg->controller->config;
	struct stm32_sai_sub_data *sub_data = dev->data;
	SAI_HandleTypeDef *hsai = &sub_data->hsai;
	uint32_t clock_source = 0U;

	if (sai_cfg->has_sai_ker_ck) {
		clock_source = sai_cfg->sai_ker_ck.bus;
	}

	switch (clock_source) {
#if defined(STM32F413xx) || defined(STM32F423xx)
	case STM32_SRC_PLLI2S_POST_R:
		hsai->Init.ClockSource = SAI_CLKSOURCE_PLLI2S;
		break;

	case STM32_SRC_PLL_POST_R:
		hsai->Init.ClockSource = SAI_CLKSOURCE_PLLR;
		break;

	case STM32_SRC_HSI:
		hsai->Init.ClockSource = SAI_CLKSOURCE_HS;
		break;
#else /* STM32F413xx || STM32F423xx */
	case STM32_SRC_PLLSAI_POST_Q:
		hsai->Init.ClockSource = SAI_CLKSOURCE_PLLSAI;
		break;

	case STM32_SRC_PLLI2S_POST_Q:
		hsai->Init.ClockSource = SAI_CLKSOURCE_PLLI2S;
		break;
#endif /* STM32F413xx || STM32F423xx */
	case 0U:
		/* No source clock defined. Do nothing. */
		break;

	default:
		LOG_ERR("Wrong source clock defined.");
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_SOC_SERIES_STM32F4X */

/* I2S/PCM protocol configuration (SAI block used with a standard audio protocol). */
static int sai_sub_i2s_pcm_conf(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
	const struct stm32_sai_sub_cfg *const sub_cfg = dev->config;
	struct stm32_sai_sub_data *const sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	SAI_HandleTypeDef *hsai = &sub_data->hsai;
	uint8_t protocol;
	uint8_t word_size;

	memcpy(&stream->i2s_cfg, i2s_cfg, sizeof(struct i2s_config));

	stream->master = true;
	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) {
		stream->master = false;
	}

#if defined(CONFIG_SOC_SERIES_STM32F4X)
	int err;

	/* ON F4x, the HAL modifies the RCC to set the source clock, so it is necessary to define
	 * the ClockSource Init parameter.
	 */
	err = stm32_sai_sub_f4_clk_src_conf(dev);
	if (err != 0) {
		return err;
	}
#endif /* CONFIG_SOC_SERIES_STM32F4X */

	hsai->Init.Synchro = SAI_ASYNCHRONOUS;

	if (dir == I2S_DIR_RX) {
		if (sub_cfg->dir == I2S_DIR_TX) {
			LOG_ERR("Invalid direction, SAI configured as TX");
			return -EINVAL;
		}

		hsai->Init.AudioMode = SAI_MODEMASTER_RX;

		if (stream->master == false) {
			hsai->Init.AudioMode = SAI_MODESLAVE_RX;
			if (sub_cfg->synchronous) {
				hsai->Init.Synchro = SAI_SYNCHRONOUS;
				LOG_WRN("Synchronous RX peripheral mode requires an active "
					"controller with bit clock gating disabled");
			}
		}

	} else if (dir == I2S_DIR_TX) {
		if (sub_cfg->dir == I2S_DIR_RX) {
			LOG_ERR("Invalid direction, SAI configured as RX");
			return -EINVAL;
		}

		hsai->Init.AudioMode = SAI_MODEMASTER_TX;

		if (stream->master == false) {
			hsai->Init.AudioMode = SAI_MODESLAVE_TX;
			if (sub_cfg->synchronous) {
				hsai->Init.Synchro = SAI_SYNCHRONOUS;
				LOG_WRN("Synchronous TX peripheral mode requires an active "
					"controller with bit clock gating disabled");
			}
		}
	} else {
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_ERR("Invalid state: %d", (int)stream->state);
		return -EINVAL;
	}

	/* MckOutput is not supported by all MCU series */
#if defined(SAI_MCK_OUTPUT_ENABLE)
	if (sub_cfg->mclk_enable && stream->master) {
		hsai->Init.MckOutput = SAI_MCK_OUTPUT_ENABLE;
	} else {
		hsai->Init.MckOutput = SAI_MCK_OUTPUT_DISABLE;
	}
#endif

	if (sub_cfg->mclk_div == (enum mclk_divider)MCLK_NO_DIV) {
		hsai->Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
	} else {
		hsai->Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;

		/* MckOverSampling is not supported by all MCU series */
#if defined(SAI_MCK_OVERSAMPLING_DISABLE)
		if (sub_cfg->mclk_div == (enum mclk_divider)MCLK_DIV_256) {
			hsai->Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
		} else {
			hsai->Init.MckOverSampling = SAI_MCK_OVERSAMPLING_ENABLE;
		}
#endif
	}

	/* AudioFrequency */
	switch (stream->i2s_cfg.frame_clk_freq) {
	case 192000U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
		break;
	case 96000U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_96K;
		break;
	case 48000U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
		break;
	case 44100U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
		break;
	case 32000U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_32K;
		break;
	case 22050U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K;
		break;
	case 16000U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
		break;
	case 11025U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_11K;
		break;
	case 8000U:
		hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_8K;
		break;
	default:
		LOG_ERR("Invalid frame_clk_freq %u", stream->i2s_cfg.frame_clk_freq);
		stream->state = I2S_STATE_NOT_READY;
		return -EINVAL;
	}

	/* WordSize */
	switch (stream->i2s_cfg.word_size) {
	case 16:
		word_size = SAI_PROTOCOL_DATASIZE_16BIT;
		stream->dma_src_size = 2;
		break;
	case 24:
		word_size = SAI_PROTOCOL_DATASIZE_24BIT;
		stream->dma_src_size = 4;
		break;
	case 32:
		word_size = SAI_PROTOCOL_DATASIZE_32BIT;
		stream->dma_src_size = 4;
		break;
	default:
		LOG_ERR("Invalid wordsize %u", stream->i2s_cfg.word_size);
		stream->state = I2S_STATE_NOT_READY;
		return -EINVAL;
	}

	/* MonoStereoMode */
	switch (stream->i2s_cfg.channels) {
	case 1:
		hsai->Init.MonoStereoMode = SAI_MONOMODE;
		LOG_DBG("SAI_MONOMODE");
		break;
	case 2:
		hsai->Init.MonoStereoMode = SAI_STEREOMODE;
		LOG_DBG("SAI_STEREOMODE");
		break;
	default:
		LOG_ERR("NOT VALID CHANNEL NUMBER %u", stream->i2s_cfg.channels);
		stream->state = I2S_STATE_NOT_READY;
		return -EINVAL;
	}

	if (stream->i2s_cfg.options & I2S_OPT_PINGPONG) {
		LOG_ERR("Ping-pong mode not supported");
		stream->state = I2S_STATE_NOT_READY;
		return -ENOTSUP;
	}

	if ((stream->i2s_cfg.format & I2S_FMT_DATA_ORDER_LSB) ||
	    (stream->i2s_cfg.format & I2S_FMT_BIT_CLK_INV) ||
	    (stream->i2s_cfg.format & I2S_FMT_FRAME_CLK_INV)) {
		LOG_ERR("Unsupported stream format");
		return -EINVAL;
	}

	switch (stream->i2s_cfg.format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		protocol = SAI_I2S_STANDARD;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		protocol = SAI_I2S_MSBJUSTIFIED;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		protocol = SAI_PCM_SHORT;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		protocol = SAI_PCM_LONG;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		protocol = SAI_I2S_LSBJUSTIFIED;
		break;
	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	/* Initialize SAI peripheral */
	if (HAL_SAI_InitProtocol(hsai, protocol, word_size, 2) != HAL_OK) {
		LOG_ERR("HAL_SAI_InitProtocol: <FAILED>");
		return -EIO;
	}

	/*
	 * TEMPORARY: compare the clock rate HAL_SAI_InitProtocol used
	 * internally (via HAL_RCCEx_GetPeriphCLKFreq) against Zephyr's own
	 * clock_control_get_rate() for the same physical clock, to check
	 * whether the two independent clock-tree implementations actually
	 * agree -- suspected root cause of a small, constant RX/TX rate
	 * mismatch (a precise ~9.4s beat period draining TX's buffer).
	 */
	{
		const struct stm32_sai_cfg *dbg_sai_cfg = sub_cfg->controller->config;
		const struct device *dbg_clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
		uint32_t dbg_rate = 0;
		int dbg_ret;

		dbg_ret = clock_control_get_rate(dbg_clk,
						  (clock_control_subsys_t)&dbg_sai_cfg->sai_ker_ck,
						  &dbg_rate);
		LOG_DBG("TX clock check: HAL Mckdiv=%u NoDivider=%u; "
			"clock_control_get_rate=%u (ret=%d)",
			hsai->Init.Mckdiv, hsai->Init.NoDivider, dbg_rate, dbg_ret);
	}

	stream->state = I2S_STATE_READY;

	/*
	 * Enable immediately SAI peripheral only when the bit clock is not gated.
	 * It allows the synchronous sub-block to become operational immediately.
	 */
	if (((i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) == 0) &&
	    hsai->Init.Synchro != SAI_SYNCHRONOUS) {

		__HAL_SAI_ENABLE(hsai);

		if (sub_cfg->dir == I2S_DIR_TX) {
			/* Prime the FIFO with a dummy sample so the controller
			 * actually starts clocking out frames.
			 */
			stm32_reg_write(&hsai->Instance->DR, 0U);
		} else {
			/* Discard whatever's in DR to clear FIFO state
			 * before the real DMA-driven reads begin.
			 */
			(void)stm32_reg_read(&hsai->Instance->DR);
		}

		LOG_DBG("SAI Enabled");
	}

	return 0;
}

/* PDM protocol configuration (SAI block A used as a PDM microphone interface). */
#if defined(CONFIG_I2S_STM32_SAI_PDM)
/*
 * Target PDM bit clock (SAI_CKn) for a given PCM sample rate, matching the
 * ST BSP's MX_SAI_ClockConfig() table. The oversampling ratio (64x or 32x)
 * differs between the two rate families so that SAI_CKn stays within the
 * range PDM MEMS microphones expect; it is not a fixed multiple of the
 * sample rate. Mckdiv itself is computed at runtime from this target and
 * the actual configured SAI kernel clock, not read from devicetree.
 */
static uint32_t sai_pdm_target_clock(uint32_t frame_clk_freq)
{
	switch (frame_clk_freq) {
	case 8000U:
	case 16000U:
	case 32000U:
		/*
		 * BSP-matched value is 1024000 (decimation=64). Doubled to
		 * 2048000 (decimation=128) as an SNR experiment: more
		 * oversampling before decimation pushes down sigma-delta
		 * quantization noise more fundamentally than post-filtering
		 * it out. Still well within typical PDM MEMS mic clock
		 * range (~1-3.25MHz). Revert to 1024000 to go back to the
		 * BSP-verified value.
		 */
		return 2048000U;
	case 48000U:
	case 96000U:
	case 192000U:
		return 3072000U;
	case 11025U:
	case 22050U:
	case 44100U:
		return 1411200U;
	case 88200U:
	case 176400U:
		return 2822400U;
	default:
		return 0U;
	}
}
#endif

#if defined(CONFIG_I2S_STM32_SAI_PDM)
static int sai_sub_pdm_conf(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
	const struct stm32_sai_sub_cfg *const sub_cfg = dev->config;
	const struct stm32_sai_cfg *sai_cfg = sub_cfg->controller->config;
	struct stm32_sai_sub_data *const sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	SAI_HandleTypeDef *hsai = &sub_data->hsai;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	const struct sai_pdm_frame_cfg *frame_cfg;
	sai_pdm_filter_fn_t filter_fn;
	uint32_t sai_ker_ck_rate;
	uint32_t target_clock;
	uint32_t decimation;
	uint32_t bytes_per_channel_per_sample;
	int ret;

	if (dir != I2S_DIR_RX) {
		LOG_ERR("SAI PDM mode only supports RX");
		return -EINVAL;
	}

	if ((i2s_cfg->options & (I2S_OPT_FRAME_CLK_TARGET | I2S_OPT_BIT_CLK_TARGET)) != 0) {
		LOG_ERR("SAI PDM mode requires the controller (master) role");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_ERR("Invalid state: %d", (int)stream->state);
		return -EINVAL;
	}

	if (sub_cfg->pdm_mic_pairs != 1U) {
		LOG_ERR("PDM decimation only supports 1 mic pair (stereo) for now");
		return -ENOTSUP;
	}

	target_clock = sai_pdm_target_clock(i2s_cfg->frame_clk_freq);
	if (target_clock == 0U) {
		LOG_ERR("Invalid frame_clk_freq %u for PDM mode", i2s_cfg->frame_clk_freq);
		return -EINVAL;
	}

	decimation = target_clock / i2s_cfg->frame_clk_freq;
	switch (decimation) {
	case 64U:
		filter_fn = Open_PDM_Filter_64;
		bytes_per_channel_per_sample = 8U;
		break;
	case 128U:
		filter_fn = Open_PDM_Filter_128;
		bytes_per_channel_per_sample = 16U;
		break;
	default:
		LOG_ERR("PDM decimation %u not supported (need 64 or 128)", decimation);
		return -ENOTSUP;
	}

	sub_data->pdm.group_pcm_samples = i2s_cfg->frame_clk_freq / 1000U;
	sub_data->pdm.group_raw_bytes =
		bytes_per_channel_per_sample * 2U * sub_data->pdm.group_pcm_samples;

	if (sub_data->pdm.group_pcm_samples == 0U ||
	    (i2s_cfg->block_size % sub_data->pdm.group_raw_bytes) != 0U) {
		LOG_ERR("block_size (%u) must be a non-zero multiple of %u raw bytes for PDM",
			i2s_cfg->block_size, sub_data->pdm.group_raw_bytes);
		return -EINVAL;
	}

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&sai_cfg->sai_ker_ck,
				      &sai_ker_ck_rate);
	if (ret != 0) {
		LOG_ERR("Could not get SAI kernel clock rate: %d", ret);
		return ret;
	}

	memcpy(&stream->i2s_cfg, i2s_cfg, sizeof(struct i2s_config));
	stream->master = true;
	stream->dma_src_size = 1; /* raw 8-bit PDM bitstream bytes, not PCM */

	frame_cfg = &sai_pdm_frame_cfg[sub_cfg->pdm_mic_pairs - 1];

	hsai->Init.Synchro = SAI_ASYNCHRONOUS;
	hsai->Init.AudioMode = SAI_MODEMASTER_RX;
	hsai->Init.Protocol = SAI_FREE_PROTOCOL;
	hsai->Init.FirstBit = SAI_FIRSTBIT_LSB;
	hsai->Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai->Init.DataSize = SAI_DATASIZE_8;
	hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
	hsai->Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
	hsai->Init.Mckdiv = DIV_ROUND_CLOSEST(sai_ker_ck_rate, 2U * target_clock);

	hsai->FrameInit.FrameLength = frame_cfg->frame_length;
	hsai->FrameInit.ActiveFrameLength = 1U;
	hsai->FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai->FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
	hsai->FrameInit.FSOffset = SAI_FS_FIRSTBIT;

	hsai->SlotInit.FirstBitOffset = 0;
	hsai->SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai->SlotInit.SlotNumber = frame_cfg->slot_number;
	hsai->SlotInit.SlotActive = SAI_SLOTACTIVE_ALL;

	hsai->Init.PdmInit.Activation = ENABLE;
	hsai->Init.PdmInit.MicPairsNbr = sub_cfg->pdm_mic_pairs;
	hsai->Init.PdmInit.ClockEnable = sub_cfg->pdm_clock_enable;

	if (HAL_SAI_Init(hsai) != HAL_OK) {
		LOG_ERR("HAL_SAI_Init (PDM): <FAILED>");
		return -EIO;
	}

	sub_data->pdm.filter_fn = filter_fn;
	sub_data->pdm.volume = sub_cfg->pdm_volume;
	sub_data->pdm.mono = sub_cfg->pdm_mono;
	sub_data->pdm.mono_channel = sub_cfg->pdm_mono_channel;

	for (int ch = 0; ch < 2; ch++) {
		TPDMFilter_InitStruct *filt = &sub_data->pdm.filter[ch];

		memset(filt, 0, sizeof(*filt));
		filt->Fs = (uint16_t)i2s_cfg->frame_clk_freq;
		filt->In_MicChannels = 2U;
		filt->Out_MicChannels = 2U;
		filt->Decimation = (uint8_t)decimation;
		filt->MaxVolume = SAI_PDM_MAX_VOLUME;
		/*
		 * HP_HZ/LP_HZ = 0 does NOT mean "disabled" here despite how it
		 * reads: Open_PDM_Filter_Init() maps HP_HZ/LP_HZ == 0 to
		 * HP_ALFA/LP_ALFA = 0, but the runtime filter never bypasses
		 * those stages -- it just freezes OldOut/OldZ at their initial
		 * 0, so the output stays 0 forever regardless of input. Both
		 * must be non-zero for any signal to reach the output.
		 */
		filt->HP_HZ = 10.0f;
		/*
		 * Was frame_clk_freq/2 (Nyquist, no real attenuation), then
		 * /4, then /8 -- each step measurably cut noise on real
		 * hardware without a corresponding gain-staging fix, so
		 * pushing the cutoff down further is the cheapest remaining
		 * SNR lever. Trades some treble for a lower noise floor.
		 */
		filt->LP_HZ = (float)i2s_cfg->frame_clk_freq / 16.0f;
		Open_PDM_Filter_Init(filt);
	}

	/*
	 * Second lowpass stage cascaded in sai_pdm_decimate(), same cutoff
	 * and same single-pole ALFA formula as the library's own LP_HZ
	 * (Open_PDM_Filter_Init()) -- two identical single-pole stages back
	 * to back give an effective 12dB/octave rolloff instead of 6dB/
	 * octave, without lowering the cutoff itself any further.
	 */
	{
		float smooth_lp_hz = (float)i2s_cfg->frame_clk_freq / 16.0f;

		sub_data->pdm.smooth_alfa =
			(uint16_t)(smooth_lp_hz * 256.0f /
				   (smooth_lp_hz + (float)i2s_cfg->frame_clk_freq / (2.0f * 3.14159f)));
		sub_data->pdm.smooth_state[0] = 0;
		sub_data->pdm.smooth_state[1] = 0;
	}

	sub_data->pdm_active = true;
	stream->state = I2S_STATE_READY;

	return 0;
}
#endif /* CONFIG_I2S_STM32_SAI_PDM */

static int stm32_sai_sub_conf(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
#if defined(CONFIG_I2S_STM32_SAI_PDM)
	const struct stm32_sai_sub_cfg *const sub_cfg = dev->config;

	if (sub_cfg->mode == SAI_MODE_PDM) {
		LOG_DBG("%s: PDM mode selected", dev->name);
		return sai_sub_pdm_conf(dev, dir, i2s_cfg);
	}
#endif

	LOG_DBG("%s: I2S/PCM mode selected", dev->name);

	return sai_sub_i2s_pcm_conf(dev, dir, i2s_cfg);
}

static int stm32_sai_sub_write(const struct device *dev, void *mem_block, size_t size)
{
	const struct stm32_sai_sub_cfg *const sub_cfg = dev->config;
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	int ret;

	if (sub_cfg->dir == I2S_DIR_RX) {
		LOG_ERR("Invalid operation, SAI configured as RX");
		return -EIO;
	}

	if (stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_READY) {
		LOG_ERR("TX Invalid state: %d", stream->state);
		return -EIO;
	}

	if (size > stream->i2s_cfg.block_size) {
		LOG_ERR("Max write size is: %zu", stream->i2s_cfg.block_size);
		return -EINVAL;
	}

	struct queue_item item = {.buffer = mem_block, .size = size};

	ret = k_msgq_put(&stream->queue, &item, K_MSEC(stream->i2s_cfg.timeout));
	if (ret < 0) {
		LOG_ERR("TX queue full");
		return ret;
	}

	return 0;
}

static int stm32_sai_sub_read(const struct device *dev, void **mem_block, size_t *size)
{
	const struct stm32_sai_sub_cfg *const sub_cfg = dev->config;
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct queue_item item;
	int ret;

	if (sub_cfg->dir == I2S_DIR_TX) {
		LOG_ERR("Invalid operation, SAI configured as TX");
		return -EIO;
	}

	if (sub_data->stream.state == I2S_STATE_NOT_READY ||
	    sub_data->stream.state == I2S_STATE_ERROR) {
		LOG_ERR("RX invalid state: %d", (int)sub_data->stream.state);
		return -EIO;
	}

	ret = k_msgq_get(&sub_data->stream.queue, &item, K_MSEC(sub_data->stream.i2s_cfg.timeout));
	if (ret < 0) {
		LOG_ERR("RX queue: %d", k_msgq_num_used_get(&sub_data->stream.queue));
		return ret;
	}

	*mem_block = item.buffer;
	*size = item.size;

	return 0;
}

static int stream_start(const struct device *dev, enum i2s_dir dir)
{
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	SAI_HandleTypeDef *hsai = &sub_data->hsai;
	struct queue_item item;
	int ret;

	if (dir == I2S_DIR_TX) {
		ret = k_msgq_get(&stream->queue, &item, K_NO_WAIT);
		if (ret < 0) {
			return -ENOMEM;
		}

		stream->mem_block = item.buffer;
		stream->mem_block_len = item.size;

		sys_cache_data_flush_range(stream->mem_block, stream->mem_block_len);

		if (HAL_SAI_Transmit_DMA(hsai, stream->mem_block,
					 stream->mem_block_len / stream->dma_src_size) != HAL_OK) {
			LOG_ERR("HAL_SAI_Transmit_DMA: <FAILED>");
			return -EIO;
		}
	} else {

		ret = k_mem_slab_alloc(stream->i2s_cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			return -ENOMEM;
		}

		stream->mem_block_len = stream->i2s_cfg.block_size;

		if (HAL_SAI_Receive_DMA(hsai, stream->mem_block,
					stream->mem_block_len / stream->dma_src_size) != HAL_OK) {
			LOG_ERR("HAL_SAI_Receive_DMA: <FAILED>");
			return -EIO;
		}
	}

	return 0;
}

static void queue_drop(const struct device *dev)
{
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	struct queue_item item;

	if (stream->mem_block != NULL) {
		stream->mem_block = NULL;
		stream->mem_block_len = 0;
	}

	while (k_msgq_get(&stream->queue, &item, K_NO_WAIT) == 0) {
		LOG_DBG("Dropping item from queue");
		k_mem_slab_free(stream->i2s_cfg.mem_slab, item.buffer);
	}
}

static int stm32_sai_sub_trigger(const struct device *dev, enum i2s_dir dir,
				 enum i2s_trigger_cmd cmd)
{
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct stream *stream = &sub_data->stream;
	unsigned int key;
	int ret;

	if (dir == I2S_DIR_BOTH) {
		LOG_ERR("Unsupported direction: %d", (int)dir);
		return -ENOTSUP;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		LOG_DBG("I2S_TRIGGER_START");

		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d", stream->state);
			return -EIO;
		}

		ret = stream->stream_start(dev, dir);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}

		stream->state = I2S_STATE_RUNNING;
		stream->last_block = false;

		break;
	case I2S_TRIGGER_STOP:
		key = irq_lock();
		LOG_DBG("I2S_TRIGGER_STOP");

		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP - Invalid state: %d", (int)stream->state);
			irq_unlock(key);
			return -EIO;
		}

		stream->last_block = true;
		stream->state = I2S_STATE_STOPPING;

		irq_unlock(key);
		break;
	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		LOG_DBG("I2S_TRIGGER_DRAIN");

		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN - Invalid state: %d", (int)stream->state);
			irq_unlock(key);
			return -EIO;
		}

		stream->state = I2S_STATE_STOPPING;

		irq_unlock(key);
		break;
	case I2S_TRIGGER_DROP:
		key = irq_lock();
		LOG_DBG("I2S_TRIGGER_DROP");

		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP - invalid state: %d", (int)stream->state);
			irq_unlock(key);
			return -EIO;
		}

		stream->queue_drop(dev);
		stream->state = I2S_STATE_READY;

		irq_unlock(key);
		break;
	case I2S_TRIGGER_PREPARE:
		key = irq_lock();
		LOG_DBG("I2S_TRIGGER_PREPARE");

		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE - invalid state: %d", (int)stream->state);
			irq_unlock(key);
			return -EIO;
		}
		stream->queue_drop(dev);
		stream->state = I2S_STATE_READY;

		irq_unlock(key);
		break;
	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}
	return 0;
}

static int sai_init(const struct device *dev)
{
	int ret = 0;

	/* Enable SAI clock */
	ret = stm32_sai_clock_en(dev);
	if (ret < 0) {
		LOG_ERR("SAI clock enable <FAILED>, ret: %d.", ret);
		return -EIO;
	}

	LOG_DBG("%s Init: <OK>", dev->name);

	return 0;
}

static const struct i2s_config *stm32_sai_sub_conf_get(const struct device *dev, enum i2s_dir dir)
{
	const struct stm32_sai_sub_cfg *const sub_cfg = dev->config;
	struct stm32_sai_sub_data *sub_data = dev->data;
	struct stream *stream = &sub_data->stream;

	if (sub_cfg->dir != dir) {
		LOG_WRN("Direction mismatch: requested %d, sub-block configured as %d", dir,
			sub_cfg->dir);
		return NULL;
	}

	if (stream != NULL && stream->state != I2S_STATE_NOT_READY) {
		return &stream->i2s_cfg;
	}

	return NULL;
}

static DEVICE_API(i2s, i2s_stm32_sai_api) = {
	.configure = stm32_sai_sub_conf,
	.trigger = stm32_sai_sub_trigger,
	.write = stm32_sai_sub_write,
	.read = stm32_sai_sub_read,
	.config_get = stm32_sai_sub_conf_get,
};

#define SAI_FIFO_THRESHOLD(node) sai_fifo_threshold[DT_ENUM_IDX(node, fifo_threshold)]
#define SAI_PDM_CLOCK_ENABLE(node) sai_pdm_clock_enable[DT_ENUM_IDX(node, st_sai_pdm_clock)]

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_dma_v2bis)
#define DMA_SLOT_BY_IDX(id, idx, slot) 0
#else
#define DMA_SLOT_BY_IDX(id, idx, slot) DT_DMAS_CELL_BY_IDX(id, idx, slot)
#endif

#define DMA_CHANNEL_CONFIG_BY_IDX(id, idx) DT_DMAS_CELL_BY_IDX(id, idx, channel_config)

#define SAI_SUB_DMA_CHANNEL_INIT(node, src, dest)                                                  \
	.stream = {                                                                                \
		.dma_dev = DEVICE_DT_GET(DT_DMAS_CTLR(node)),                                      \
		.dma_channel = DT_DMAS_CELL_BY_IDX(node, 0, channel),                              \
		.reg = (DMA_TypeDef *)DT_REG_ADDR(DT_PHANDLE_BY_IDX(node, dmas, 0)),               \
		.dma_cfg = {                                                                       \
			.dma_slot = DMA_SLOT_BY_IDX(node, 0, slot),                                \
			.channel_direction = src##_TO_##dest,                                      \
			.dma_callback = dma_callback,                                              \
			.channel_priority = STM32_DMA_CONFIG_PRIORITY(                             \
				DMA_CHANNEL_CONFIG_BY_IDX(node, 0)),                               \
			.source_data_size = STM32_DMA_CONFIG_##src##_DATA_SIZE(                    \
				DMA_CHANNEL_CONFIG_BY_IDX(node, 0)),                               \
			.dest_data_size = STM32_DMA_CONFIG_##dest##_DATA_SIZE(                     \
				DMA_CHANNEL_CONFIG_BY_IDX(node, 0)),                               \
		},                                                                                 \
		.stream_start = stream_start,                                                      \
		.queue_drop = queue_drop,                                                          \
	}

#define SAI_SUB_MODE(node) CONCAT(SAI_MODE_, DT_STRING_UPPER_TOKEN(node, st_sai_mode))

#define SAI_SUB_INIT(node)                                                                         \
	BUILD_ASSERT(SAI_SUB_MODE(node) != SAI_MODE_PDM ||                                        \
			     !DT_DMAS_HAS_NAME(node, tx),                                          \
		     "SAI PDM mode requires RX DMA only (dma-names must be \"rx\")");             \
	BUILD_ASSERT(SAI_SUB_MODE(node) != SAI_MODE_PDM ||                                        \
			     DT_PROP(node, st_sai_pdm_capable),                                    \
		     DT_NODE_FULL_NAME(node) " does not support PDM mode");                        \
	BUILD_ASSERT(SAI_SUB_MODE(node) != SAI_MODE_PDM ||                                        \
			     IS_ENABLED(CONFIG_I2S_STM32_SAI_PDM),                                 \
		     "PDM mode requires CONFIG_I2S_STM32_SAI_PDM=y (PDM2PCM decimation)");         \
	BUILD_ASSERT(SAI_SUB_MODE(node) != SAI_MODE_PDM ||                                        \
			     DT_PROP(node, st_sai_pdm_volume) <= 64,                               \
		     "st,sai-pdm-volume must be 0-64");                                            \
	PINCTRL_DT_DEFINE(node);                                                                   \
                                                                                                   \
	static struct stm32_sai_sub_data sub_data_##node = {                                       \
		.hsai = {                                                                          \
			.Instance = (SAI_Block_TypeDef *)DT_REG_ADDR(node),                        \
			.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE,                               \
			.Init.FIFOThreshold = SAI_FIFO_THRESHOLD(node),                            \
			.Init.SynchroExt = SAI_SYNCEXT_DISABLE,                                    \
			.Init.CompandingMode = SAI_NOCOMPANDING,                                   \
			.Init.TriState = SAI_OUTPUT_NOTRELEASED,                                   \
		},                                                                                 \
		COND_CODE_1(DT_DMAS_HAS_NAME(node, tx),                                            \
		    (SAI_SUB_DMA_CHANNEL_INIT(node, MEMORY, PERIPHERAL)),                          \
		    (SAI_SUB_DMA_CHANNEL_INIT(node, PERIPHERAL, MEMORY))),                         \
	};                                                                                         \
                                                                                                   \
	static const struct stm32_sai_sub_cfg sub_cfg_##node = {                                   \
		.pincfg = PINCTRL_DT_DEV_CONFIG_GET(node),                                         \
		.mclk_enable = DT_PROP(node, mclk_enable),                                         \
		.mclk_div = DT_ENUM_IDX(node, mclk_divider),                                       \
		.synchronous = DT_PROP(node, synchronous),                                         \
		.mode = SAI_SUB_MODE(node),                                                        \
		.pdm_mic_pairs = DT_PROP(node, st_sai_pdm_mic_pairs),                              \
		.pdm_clock_enable = SAI_PDM_CLOCK_ENABLE(node),                                    \
		.pdm_volume = DT_PROP(node, st_sai_pdm_volume),                                    \
		.pdm_mono = DT_PROP(node, st_sai_pdm_mono),                                        \
		.pdm_mono_channel = DT_ENUM_IDX(node, st_sai_pdm_mono_channel),                    \
		.controller = DEVICE_DT_GET(DT_PARENT(node)),                                      \
		.dir = COND_CODE_1(DT_DMAS_HAS_NAME(node, tx), (I2S_DIR_TX), (I2S_DIR_RX)),        \
	};                                                                                         \
	DEVICE_DT_DEFINE(node, &sai_sub_init, NULL, &sub_data_##node, &sub_cfg_##node,             \
			 POST_KERNEL, CONFIG_I2S_INIT_PRIORITY, &i2s_stm32_sai_api);               \
	K_MSGQ_DEFINE_STATIC_TYPE(queue_##node, struct queue_item,                                 \
				  CONFIG_I2S_STM32_SAI_BLOCK_COUNT);

#define SAI_KER_CK_FIELD_INIT(inst, n)                                                             \
	COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(inst, n),                                              \
		(.n = STM32_DT_INST_CLOCK_INFO_BY_NAME(inst, n), .has_##n = true,),                \
		(.has_##n = false,))

#define SAI_KER_CK_INIT(inst)                                                                      \
	SAI_KER_CK_FIELD_INIT(inst, sai_ker_ck)                                                    \
	SAI_KER_CK_FIELD_INIT(inst, sai_b_ker_ck)

/* Controller Node */
#define SAI_INIT(inst)                                                                             \
	static const struct stm32_sai_cfg sai_cfg_##inst = {                                       \
		.sai_ck = STM32_DT_INST_CLOCK_INFO_BY_NAME(inst, sai_ck),                          \
		SAI_KER_CK_INIT(inst)                                                              \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &sai_init, NULL, NULL, &sai_cfg_##inst, POST_KERNEL,           \
			      CONFIG_I2S_INIT_PRIORITY, NULL);                                     \
                                                                                                   \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, SAI_SUB_INIT)

DT_INST_FOREACH_STATUS_OKAY(SAI_INIT)
