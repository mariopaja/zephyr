/*
 * Copyright (c) 2025 Mario Paja
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#define DT_DRV_COMPAT st_stm32_xspi_psram_aps6404

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/multi_heap/shared_multi_heap.h>

LOG_MODULE_REGISTER(memc_stm32_xspi_psram_aps6404, 4);

#define STM32_XSPI_NODE DT_INST_PARENT(0)

#ifdef CONFIG_SHARED_MULTI_HEAP
struct shared_multi_heap_region smh_psram = {
	.addr = DT_REG_ADDR(DT_NODELABEL(psram)),
	.size = DT_REG_SIZE(DT_NODELABEL(psram)),
	.attr = SMH_REG_ATTR_EXTERNAL,
};
#endif

#define STM32_XSPI_CLOCK_PRESCALER_MIN                0U
#define STM32_XSPI_CLOCK_PRESCALER_MAX                255U
#define STM32_XSPI_CLOCK_COMPUTE(bus_freq, prescaler) ((bus_freq) / ((prescaler) + 1U))

struct memc_stm32_xspi_psram_config {
	const struct pinctrl_dev_config *pcfg;
	const struct stm32_pclken pclken;
	const struct stm32_pclken pclken_ker;
	const struct stm32_pclken pclken_mgr;
	size_t memory_size;
	bool qspi_enable;
	uint32_t max_frequency;
};

struct memc_stm32_xspi_psram_data {
	XSPI_HandleTypeDef hxspi;
};

static int ap_memory_reset(XSPI_HandleTypeDef *hxspi)
{
	XSPI_RegularCmdTypeDef cmd = {0};

	/* Initialize the write register command.
	 * The following fields are already set to 0 thanks to cmd = {0}.
	 * They are kept in comment for better understanding of the command.
	 * cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	 * cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	 * cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
	 * cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
	 * cmd.Address = NULL;
	 * cmd.DataMode = HAL_XSPI_DATA_NONE;
	 * cmd.DataLength = 0U;
	 * cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
	 * cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
	 * cmd.DummyCycles = 0U;
	 * cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
	 * cmd.AddressWidth = HAL_XSPI_ADDRESS_8_BITS;
	 * cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	 */
	cmd.Instruction = 0x66;
	cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;

	/* Configure the command */
	if (HAL_XSPI_Command(hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI write command failed");
		return -EIO;
	} else {
		LOG_DBG("XSPI write command ok");
	}

	cmd.Instruction = 0x99;
	/* Configure the command */
	if (HAL_XSPI_Command(hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI write command failed");
		return -EIO;
	} else {
		LOG_DBG("XSPI write command ok");
	}

	return 0;
}

static int ap_memory_read_id(XSPI_HandleTypeDef *hxspi, uint8_t *pID)
{
	XSPI_RegularCmdTypeDef cmd = {0};

	/* Initialize the write register command.
	 * The following fields are already set to 0 thanks to cmd = {0}.
	 * They are kept in comment for better understanding of the command.
	 * cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	 * cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
	 * cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	 * cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
	 * cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
	 * cmd.Address = NULL;
	 * cmd.DataMode = HAL_XSPI_DATA_NONE;
	 * cmd.DataLength = 0U;
	 * cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
	 * cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
	 * cmd.DummyCycles = 0U;
	 * cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
	 * cmd.AddressWidth = HAL_XSPI_ADDRESS_8_BITS;
	 * cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	 */
	cmd.Instruction = 0x9F;
	cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
	cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
	cmd.DataMode = HAL_XSPI_DATA_1_LINE;
	cmd.DataLength = 2U;

	/* Configure the command */
	if (HAL_XSPI_Command(hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI write command failed");
		return -EIO;
	} else {
		LOG_DBG("XSPI write command ok");
	}

	return HAL_XSPI_Receive(hxspi, pID, HAL_TIMEOUT);
	;
}

static int memc_stm32_xspi_psram_init(const struct device *dev)
{
	const struct memc_stm32_xspi_psram_config *dev_cfg = dev->config;
	struct memc_stm32_xspi_psram_data *dev_data = dev->data;
	XSPI_HandleTypeDef hxspi = dev_data->hxspi;
	uint32_t ahb_clock_freq;
	XSPIM_CfgTypeDef cfg = {0};
	XSPI_RegularCmdTypeDef cmd = {0};
	XSPI_MemoryMappedTypeDef mem_mapped_cfg = {0};
	uint32_t prescaler = STM32_XSPI_CLOCK_PRESCALER_MIN;
	int ret;

	/* Signals configuration */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("XSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t)&dev_cfg->pclken) != 0) {
		LOG_ERR("Could not enable XSPI clock");
		return -EIO;
	}
	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				   (clock_control_subsys_t)&dev_cfg->pclken, &ahb_clock_freq) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken)");
		return -EIO;
	}

#if DT_CLOCKS_HAS_NAME(STM32_XSPI_NODE, xspi_ker)
	/* Kernel clock config for peripheral if any */
	if (clock_control_configure(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				    (clock_control_subsys_t)&dev_cfg->pclken_ker, NULL) != 0) {
		LOG_ERR("Could not select XSPI domain clock");
		return -EIO;
	}

	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				   (clock_control_subsys_t)&dev_cfg->pclken_ker,
				   &ahb_clock_freq) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken_ker)");
		return -EIO;
	}
#endif

#if DT_CLOCKS_HAS_NAME(STM32_XSPI_NODE, xspi_mgr)
	/* Clock domain corresponding to the IO-Mgr (XSPIM) */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t)&dev_cfg->pclken_mgr) != 0) {
		LOG_ERR("Could not enable XSPI Manager clock");
		return -EIO;
	}
#endif

	for (; prescaler <= STM32_XSPI_CLOCK_PRESCALER_MAX; prescaler++) {
		uint32_t clk = STM32_XSPI_CLOCK_COMPUTE(ahb_clock_freq, prescaler);

		if (clk <= dev_cfg->max_frequency) {
			break;
		}
	}

	if (prescaler > STM32_XSPI_CLOCK_PRESCALER_MAX) {
		LOG_ERR("XSPI could not find valid prescaler value");
		return -EINVAL;
	}

	hxspi.Init.ClockPrescaler = prescaler;
	LOG_DBG("ClockPrescaler: %d", hxspi.Init.ClockPrescaler);
	hxspi.Init.MemorySize = find_msb_set(dev_cfg->memory_size) - 2;

	if (HAL_XSPI_Init(&hxspi) != HAL_OK) {
		LOG_ERR("XSPI Init failed");
		return -EIO;
	} else {
		LOG_DBG("XSPI Init ok");
	}

	cfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
	cfg.IOPort = HAL_XSPIM_IOPORT_1;

	if (HAL_XSPIM_Config(&hxspi, &cfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPIMgr Init failed");
		return -EIO;
	} else {
		LOG_DBG("XSPIMgr Init ok");
	}

	/* Memory Reset */
	ap_memory_reset(&hxspi);

	k_msleep(300);

	/* Read Memory ID */
	uint8_t id[2];
	ret = ap_memory_read_id(&hxspi, id);
	if (ret != 0) {
		LOG_ERR("XSPI read ID failed");
	} else {
		LOG_DBG("PSRAM ID: 0x%02x, 0x%02x", id[0], id[1]);
	}

	/* Memory QSPI Enable */
	if (dev_cfg->qspi_enable) {
		/* Memory Mapped Write */
		cmd.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
		cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
		cmd.Instruction = 0x38;
		cmd.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
		cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
		cmd.DataMode = HAL_XSPI_DATA_4_LINES;
		cmd.DQSMode = HAL_XSPI_DQS_ENABLE;

		if (HAL_XSPI_Command(&hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			LOG_ERR("XSPI write command failed");
		} else {
			LOG_DBG("XSPI write command ok");
		}

		/* Memory Mapped Read */
		cmd.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
		cmd.Instruction = 0xEB;
		cmd.DummyCycles = 6U;
		cmd.DQSMode = HAL_XSPI_DQS_DISABLE;

		if (HAL_XSPI_Command(&hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			LOG_ERR("XSPI write command failed");
		} else {
			LOG_DBG("XSPI write command ok");
		}

	} else {
		/* Memory Mapped Write */
		cmd.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
		cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
		cmd.Instruction = 0x02;
		cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
		cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
		cmd.DataMode = HAL_XSPI_DATA_1_LINE;
		cmd.DQSMode = HAL_XSPI_DQS_ENABLE;

		if (HAL_XSPI_Command(&hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			LOG_ERR("XSPI write command failed");
		} else {
			LOG_DBG("XSPI write command ok");
		}

		/* Memory Mapped Read */
		cmd.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
		cmd.Instruction = 0x0B;
		cmd.DummyCycles = 8U;
		cmd.DQSMode = HAL_XSPI_DQS_DISABLE;

		if (HAL_XSPI_Command(&hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			LOG_ERR("XSPI write command failed");
		} else {
			LOG_DBG("XSPI write command ok");
		}
	}

	mem_mapped_cfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;

#if defined(XSPI_CR_NOPREF)
	mem_mapped_cfg.NoPrefetchData = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
#endif
#if defined(XSPI_CR_NOPREF_AXI)
	mem_mapped_cfg.NoPrefetchAXI = HAL_XSPI_AXI_PREFETCH_DISABLE;
#endif

	if (HAL_XSPI_MemoryMapped(&hxspi, &mem_mapped_cfg) != HAL_OK) {
		LOG_ERR("XSPI memory mapped failed");
		return -EIO;
	} else {
		LOG_DBG("XSPI memory mapped ok");
	}
#if defined(XSPI_CR_NOPREF)
	MODIFY_REG(hxspi.Instance->CR, XSPI_CR_NOPREF, HAL_XSPI_AUTOMATIC_PREFETCH_DISABLE);
#endif

#ifdef CONFIG_SHARED_MULTI_HEAP
	shared_multi_heap_pool_init();
	ret = shared_multi_heap_add(&smh_psram, NULL);
	if (ret < 0) {
		return ret;
	}
#endif

	return 0;
}

PINCTRL_DT_DEFINE(STM32_XSPI_NODE);

static const struct memc_stm32_xspi_psram_config memc_stm32_xspi_cfg = {
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(STM32_XSPI_NODE),
	.pclken = {.bus = DT_CLOCKS_CELL_BY_NAME(STM32_XSPI_NODE, xspix, bus),
		   .enr = DT_CLOCKS_CELL_BY_NAME(STM32_XSPI_NODE, xspix, bits)},
#if DT_CLOCKS_HAS_NAME(STM32_XSPI_NODE, xspi_ker)
	.pclken_ker = {.bus = DT_CLOCKS_CELL_BY_NAME(STM32_XSPI_NODE, xspi_ker, bus),
		       .enr = DT_CLOCKS_CELL_BY_NAME(STM32_XSPI_NODE, xspi_ker, bits)},
#endif
#if DT_CLOCKS_HAS_NAME(STM32_XSPI_NODE, xspi_mgr)
	.pclken_mgr = {.bus = DT_CLOCKS_CELL_BY_NAME(STM32_XSPI_NODE, xspi_mgr, bus),
		       .enr = DT_CLOCKS_CELL_BY_NAME(STM32_XSPI_NODE, xspi_mgr, bits)},
#endif
	.memory_size = DT_INST_PROP(0, size) / 8, /* In Bytes */
	.max_frequency = DT_INST_PROP(0, max_frequency),
	.qspi_enable = DT_INST_PROP(0, qspi_enable),
};

static struct memc_stm32_xspi_psram_data memc_stm32_xspi_data = {
	.hxspi =
		{
			.Instance = (XSPI_TypeDef *)DT_REG_ADDR(STM32_XSPI_NODE),
			.Init =
				{
					.FifoThresholdByte = 1U,
					.MemoryMode = HAL_XSPI_SINGLE_MEM,
					.MemoryType = HAL_XSPI_MEMTYPE_APMEM,
					.ChipSelectHighTimeCycle = 1U,
					.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE,
					.ClockMode = HAL_XSPI_CLOCK_MODE_0,
					.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED,
					.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE,
					.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE,
					.ChipSelectBoundary = DT_INST_PROP(0, st_csbound),
					.MaxTran = 0U,
					.Refresh = 0U,
					.MemorySelect = HAL_XSPI_CSSEL_NCS1,
				},
		},
};

DEVICE_DT_INST_DEFINE(0, &memc_stm32_xspi_psram_init, NULL, &memc_stm32_xspi_data,
		      &memc_stm32_xspi_cfg, POST_KERNEL, CONFIG_MEMC_INIT_PRIORITY, NULL);
