/* LAN9250 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2024 Mario Paja
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT microchip_lan9250

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>
#if defined(CONFIG_PTP_CLOCK_LAN9250)
#include <zephyr/drivers/ptp_clock.h>
#endif

#include "eth_lan9250_priv.h"

LOG_MODULE_REGISTER(eth_lan9250, CONFIG_ETHERNET_LOG_LEVEL);

#define LAN9250_RX_DATA_OFFSET 4U
#define LAN9250_RX_MIN_LEN     (sizeof(struct net_eth_hdr) + LAN9250_CRC_LEN)
#define LAN9250_RX_MAX_LEN     (NET_ETH_MAX_FRAME_SIZE + LAN9250_CRC_LEN)

static int lan9250_write_sys_reg(const struct device *dev, uint16_t address, uint32_t data)
{
	const struct lan9250_config *config = dev->config;
	uint8_t cmd[1] = {LAN9250_SPI_INSTR_WRITE};
	uint8_t addr[2];
	uint8_t instr[4];
	struct spi_buf tx_buf[3];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 3};

	sys_put_be16(address, addr);
	sys_put_le32(data, instr);

	tx_buf[0].buf = &cmd;
	tx_buf[0].len = ARRAY_SIZE(cmd);
	tx_buf[1].buf = addr;
	tx_buf[1].len = ARRAY_SIZE(addr);
	tx_buf[2].buf = instr;
	tx_buf[2].len = ARRAY_SIZE(instr);

	return spi_write_dt(&config->spi, &tx);
}

/* Read data using the SPI Read instruction (up to 30 MHz) or, if enabled,
 * the Fast Read instruction (up to 80 MHz), which needs one dummy byte.
 */
static int lan9250_read(const struct device *dev, uint16_t address, uint8_t *data, size_t len)
{
	const struct lan9250_config *config = dev->config;
	const bool fast = IS_ENABLED(CONFIG_ETH_LAN9250_SPI_FAST_READ);
	/* Instruction, address and optional dummy byte */
	uint8_t hdr[4] = {fast ? LAN9250_SPI_INSTR_FAST_READ : LAN9250_SPI_INSTR_READ};
	size_t hdr_len = fast ? 4 : 3;
	struct spi_buf tx_buf[2];
	struct spi_buf rx_buf[2];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 2};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = 2};

	sys_put_be16(address, &hdr[1]);

	tx_buf[0].buf = hdr;
	tx_buf[0].len = hdr_len;
	tx_buf[1].buf = NULL;
	tx_buf[1].len = len;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = hdr_len;
	rx_buf[1].buf = data;
	rx_buf[1].len = len;

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

static int lan9250_read_sys_reg(const struct device *dev, uint16_t address, uint32_t *value)
{
	uint8_t data[4];
	int ret;

	ret = lan9250_read(dev, address, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}

	*value = sys_get_le32(data);

	return 0;
}

static int lan9250_wait_ready(const struct device *dev, uint16_t address, uint32_t mask,
			      uint32_t expected, uint32_t m_second)
{
	int ret;
	uint32_t tmp;
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(m_second));

	while (true) {
		ret = lan9250_read_sys_reg(dev, address, &tmp);
		if ((ret == 0) && ((tmp & mask) == expected)) {
			return 0;
		}
		if (sys_timepoint_expired(end)) {
			return -EIO;
		}
		k_msleep(1);
	}
}

static int lan9250_read_mac_reg(const struct device *dev, uint8_t address, uint32_t *value)
{
	uint32_t tmp;
	int ret;

	/* Wait for MAC to be ready and send writing register command and data */
	ret = lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_MAC_CSR_CMD,
				    address | LAN9250_MAC_CSR_CMD_BUSY | LAN9250_MAC_CSR_CMD_READ);
	if (ret < 0) {
		return ret;
	}

	/* Wait for MAC to be ready and send writing register command and data */
	ret = lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_read_sys_reg(dev, LAN9250_MAC_CSR_DATA, &tmp);
	if (ret < 0) {
		return ret;
	}

	*value = tmp;

	return 0;
}

static int lan9250_write_mac_reg(const struct device *dev, uint8_t address, uint32_t data)
{
	int ret;
	/* Wait for MAC to be ready and send writing register command and data */
	ret = lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_MAC_CSR_DATA, data);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_MAC_CSR_CMD, address | LAN9250_MAC_CSR_CMD_BUSY);
	if (ret < 0) {
		return ret;
	}

	/* Wait until writing MAC is done */
	return lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				  LAN9250_MAC_TIMEOUT);
}

static int lan9250_wait_mac_ready(const struct device *dev, uint8_t address, uint32_t mask,
				  uint32_t expected, uint32_t m_second)
{
	int ret;
	uint32_t tmp;
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(m_second));

	while (true) {
		ret = lan9250_read_mac_reg(dev, address, &tmp);
		if ((ret == 0) && ((tmp & mask) == expected)) {
			return 0;
		}
		if (sys_timepoint_expired(end)) {
			return -EIO;
		}
		k_msleep(1);
	}
}

static int lan9250_read_phy_reg(const struct device *dev, uint8_t address, uint16_t *value)
{
	uint32_t tmp;
	int ret;

	/* Wait PHY to be ready and send reading register command */
	ret = lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				     LAN9250_PHY_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	/* Reference: Microchip Ethernet LAN9250
	 * https://github.com/microchip-pic-avr-solutions/ethernet-lan9250/
	 *
	 * Datasheet:
	 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/00001913A.pdf
	 *
	 * 12.2.18 PHY REGISTERS
	 * The PHY registers are indirectly accessed through the Host MAC MII Access Register
	 * (HMAC_MII_ACC) and Host MAC MII Data Register (HMAC_MII_DATA).
	 *
	 * Write 32bit value to the indirect MAC registers
	 * Where phy_add = 0b00001 & index = address
	 * Data = ((phy_add & 0x1F) << 11) | ((index & 0x1F) << 6)
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_MII_ACC, (1 << 11) | ((address & 0x1F) << 6));
	if (ret < 0) {
		return ret;
	}

	/* Wait PHY to be ready and send reading register command */
	ret = lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				     LAN9250_PHY_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	/* Read 32bit value from the indirect MAC registers */
	ret = lan9250_read_mac_reg(dev, LAN9250_HMAC_MII_DATA, &tmp);
	if (ret < 0) {
		return ret;
	}

	*value = tmp;

	return 0;
}

static int lan9250_write_phy_reg(const struct device *dev, uint8_t address, uint16_t data)
{
	int ret;
	/* Wait PHY to be ready and send reading register command */
	ret = lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				     LAN9250_PHY_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_MII_DATA, data);
	if (ret < 0) {
		return ret;
	}

	/* Reference: Microchip Ethernet LAN9250
	 * https://github.com/microchip-pic-avr-solutions/ethernet-lan9250/
	 *
	 * Datasheet:
	 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/00001913A.pdf
	 *
	 * 12.2.18 PHY REGISTERS
	 * The PHY registers are indirectly accessed through the Host MAC MII Access Register
	 * (HMAC_MII_ACC) and Host MAC MII Data Register (HMAC_MII_DATA).
	 *
	 * Write 32bit value to the indirect MAC registers
	 * Where phy_add = 0b00001 & index = address
	 * Data = ((phy_add & 0x1F) << 11) | ((index & 0x1F)<< 6) | MIIWnR
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_MII_ACC,
				    (1 << 11) | ((address & 0x1F) << 6) |
					    LAN9250_HMAC_MII_ACC_MIIW_R);
	if (ret < 0) {
		return ret;
	}

	/* Wait PHY to be ready and send reading register command */
	return lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				      LAN9250_PHY_TIMEOUT);
}

static int lan9250_set_macaddr(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;
	int ret;

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_ADDRL,
				    ctx->mac_address[0] | (ctx->mac_address[1] << 8) |
					    (ctx->mac_address[2] << 16) |
					    (ctx->mac_address[3] << 24));
	if (ret < 0) {
		return ret;
	}

	return lan9250_write_mac_reg(dev, LAN9250_HMAC_ADDRH,
				     ctx->mac_address[4] | (ctx->mac_address[5] << 8));
}

static int lan9250_sw_reset(const struct device *dev)
{
	int ret;

	ret = lan9250_write_sys_reg(dev, LAN9250_RESET_CTL,
				    LAN9250_RESET_CTL_HMAC_RST | LAN9250_RESET_CTL_PHY_RST |
					    LAN9250_RESET_CTL_DIGITAL_RST);
	if (ret < 0) {
		return ret;
	}

	/* Wait until LAN9250 SPI bus is ready */
	return lan9250_wait_ready(dev, LAN9250_BYTE_TEST, BOTR_MASK, LAN9250_BYTE_TEST_DEFAULT,
				  LAN9250_RESET_TIMEOUT);
}

#if defined(CONFIG_PTP_CLOCK_LAN9250)
/* PTP event messages: Sync, Delay_Req, Pdelay_Req and Pdelay_Resp */
#define LAN9250_PTP_EVENT_MSG_MASK  0x000FU
#define LAN9250_PTP_VERSION         2U
#define LAN9250_PTP_EVENT_PORT      319U
#define LAN9250_PTP_HDR_LEN         34U
#define LAN9250_PTP_HDR_SEQ_ID      30U
#define LAN9250_PTP_MAX_VLAN_TAGS   2
#define LAN9250_1588_TS_CAPTURES    4
#define LAN9250_1588_NS_MASK        0x3FFFFFFFU

static int lan9250_1588_init(const struct device *dev)
{
	const uint32_t ts_config = LAN9250_1588_TIMESTAMP_CONFIG_VERSION(LAN9250_PTP_VERSION) |
				   LAN9250_1588_TIMESTAMP_CONFIG_MSG_EN(LAN9250_PTP_EVENT_MSG_MASK);
	int ret;

	/* The 1588 configuration must not change while the unit is enabled */
	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_DISABLE);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_wait_ready(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_ENABLE, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	/* Timestamp received and transmitted PTPv2 event messages. Layer 2,
	 * UDP/IPv4 and UDP/IPv6 messages to the PTP multicast addresses are
	 * detected by default.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_1588_BANK_SEL, LAN9250_1588_BANK_SEL_RX);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_RX_TIMESTAMP_CONFIG, ts_config);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_BANK_SEL, LAN9250_1588_BANK_SEL_TX);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_TX_TIMESTAMP_CONFIG, ts_config);
	if (ret < 0) {
		return ret;
	}

	return lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_ENABLE);
}

/* Return the offset of the PTP header in a frame carrying a PTPv2 event
 * message, or -ENOENT. Layer 2 messages and UDP messages over IPv4 or over
 * IPv6 without extension headers are recognized, with up to two VLAN tags.
 */
static int lan9250_ptp_hdr_offset(const uint8_t *frame, size_t len)
{
	size_t off = offsetof(struct net_eth_hdr, type);
	uint16_t type;
	bool udp = false;

	if (len < sizeof(struct net_eth_hdr)) {
		return -ENOENT;
	}

	type = sys_get_be16(&frame[off]);
	off += sizeof(uint16_t);

	for (int i = 0; (i < LAN9250_PTP_MAX_VLAN_TAGS) && (type == NET_ETH_PTYPE_VLAN); i++) {
		if (len < off + 2 * sizeof(uint16_t)) {
			return -ENOENT;
		}

		type = sys_get_be16(&frame[off + sizeof(uint16_t)]);
		off += 2 * sizeof(uint16_t);
	}

	switch (type) {
	case NET_ETH_PTYPE_PTP:
		break;
	case NET_ETH_PTYPE_IP:
		if ((len < off + NET_IPV4H_LEN) || (frame[off + 9] != NET_IPPROTO_UDP)) {
			return -ENOENT;
		}

		off += (frame[off] & 0x0F) * sizeof(uint32_t);
		udp = true;
		break;
	case NET_ETH_PTYPE_IPV6:
		if ((len < off + NET_IPV6H_LEN) || (frame[off + 6] != NET_IPPROTO_UDP)) {
			return -ENOENT;
		}

		off += NET_IPV6H_LEN;
		udp = true;
		break;
	default:
		return -ENOENT;
	}

	if (udp) {
		/* Event messages use the event port */
		if ((len < off + NET_UDPH_LEN) ||
		    (sys_get_be16(&frame[off + sizeof(uint16_t)]) != LAN9250_PTP_EVENT_PORT)) {
			return -ENOENT;
		}

		off += NET_UDPH_LEN;
	}

	if ((len < off + LAN9250_PTP_HDR_LEN) ||
	    ((frame[off + 1] & 0x0F) != LAN9250_PTP_VERSION) ||
	    ((BIT(frame[off] & 0x0F) & LAN9250_PTP_EVENT_MSG_MASK) == 0)) {
		return -ENOENT;
	}

	return off;
}

/* Read the timestamp of a PTP event message from the RX or TX capture
 * buffer. Captures that do not match the message type and sequence ID
 * belong to frames that were dropped, or not timestamped by the network
 * stack, and are discarded. Returns -ENOENT if no capture matches.
 */
static int lan9250_1588_get_ts(const struct device *dev, bool tx, const uint8_t *ptp_hdr,
			       struct net_ptp_time *ts)
{
	const uint32_t sts_bit = tx ? LAN9250_1588_INT_STS_TX_TS : LAN9250_1588_INT_STS_RX_TS;
	const uint16_t address = tx ? LAN9250_1588_TX_EGRESS_SEC : LAN9250_1588_RX_INGRESS_SEC;
	const uint32_t expected =
		FIELD_PREP(LAN9250_1588_MSG_HEADER_MSG_TYPE, ptp_hdr[0] & 0x0F) |
		sys_get_be16(&ptp_hdr[LAN9250_PTP_HDR_SEQ_ID]);
	/* Timestamp seconds, timestamp nanoseconds and message header */
	uint8_t data[3 * sizeof(uint32_t)];
	uint32_t header;
	uint32_t sts;
	int ret;

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_BANK_SEL,
				    tx ? LAN9250_1588_BANK_SEL_TX : LAN9250_1588_BANK_SEL_RX);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < LAN9250_1588_TS_CAPTURES; i++) {
		ret = lan9250_read_sys_reg(dev, LAN9250_1588_INT_STS, &sts);
		if (ret < 0) {
			return ret;
		}

		if ((sts & sts_bit) == 0) {
			return -ENOENT;
		}

		ret = lan9250_read(dev, LAN9250_SPI_ADDR_INC | address, data, sizeof(data));
		if (ret < 0) {
			return ret;
		}

		/* Release the capture, the next one is loaded if available */
		ret = lan9250_write_sys_reg(dev, LAN9250_1588_INT_STS, sts_bit);
		if (ret < 0) {
			return ret;
		}

		header = sys_get_le32(&data[8]);
		if ((header & LAN9250_1588_MSG_HEADER_ID) == expected) {
			ts->second = sys_get_le32(&data[0]);
			ts->nanosecond = sys_get_le32(&data[4]) & LAN9250_1588_NS_MASK;
			return 0;
		}

		LOG_DBG("Discarding %s timestamp, header 0x%08x", tx ? "TX" : "RX", header);
	}

	return -ENOENT;
}

static void lan9250_1588_rx_ts(const struct device *dev, struct net_pkt *pkt, size_t len)
{
	struct lan9250_runtime *ctx = dev->data;
	struct net_ptp_time ts;
	int off;
	int ret;

	off = lan9250_ptp_hdr_offset(ctx->buf, len);
	if (off < 0) {
		return;
	}

	ret = lan9250_1588_get_ts(dev, false, &ctx->buf[off], &ts);
	if (ret < 0) {
		LOG_DBG("No RX timestamp: %d", ret);
		return;
	}

	net_pkt_set_timestamp(pkt, &ts);
	net_pkt_set_rx_timestamping(pkt, true);
}

/* Wait for the egress timestamp of a frame still in ctx->buf */
static void lan9250_1588_tx_ts(const struct device *dev, struct net_pkt *pkt, size_t len)
{
	struct lan9250_runtime *ctx = dev->data;
	struct net_ptp_time ts;
	k_timepoint_t end;
	int off;
	int ret;

	off = lan9250_ptp_hdr_offset(ctx->buf, len);
	if (off < 0) {
		LOG_DBG("TX frame not timestamped by the device");
		return;
	}

	end = sys_timepoint_calc(K_MSEC(LAN9250_1588_TX_TS_TIMEOUT));
	while (true) {
		ret = lan9250_1588_get_ts(dev, true, &ctx->buf[off], &ts);
		if (ret == 0) {
			net_pkt_set_timestamp(pkt, &ts);
			net_if_add_tx_timestamp(pkt);
			return;
		}

		if (ret != -ENOENT) {
			LOG_ERR("Reading TX timestamp failed: %d", ret);
			return;
		}

		if (sys_timepoint_expired(end)) {
			LOG_WRN("No TX timestamp");
			return;
		}

		k_usleep(LAN9250_1588_TX_TS_POLL_US);
	}
}
#endif /* CONFIG_PTP_CLOCK_LAN9250 */

static int lan9250_configure(const struct device *dev)
{
	uint32_t tmp;
	int ret;

	ret = lan9250_wait_ready(dev, LAN9250_HW_CFG, LAN9250_HW_CFG_DEVICE_READY,
				 LAN9250_HW_CFG_DEVICE_READY, LAN9250_RESET_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Device not ready");
		return ret;
	}

	/* Read LAN9250 hardware ID */
	ret = lan9250_read_sys_reg(dev, LAN9250_ID_REV, &tmp);
	if (ret < 0) {
		return ret;
	}

	if ((tmp & LAN9250_ID_REV_CHIP_ID) != LAN9250_ID_REV_CHIP_ID_DEFAULT) {
		LOG_ERR("Bad Rev ID: %08x", tmp);
		return -ENODEV;
	}

	/* Configure TX FIFO size mode to be 8:
	 *
	 *   - TX data FIFO size:   7680
	 *   - RX data FIFO size:   7680
	 *   - TX status FIFO size: 512
	 *   - RX status FIFO size: 512
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_HW_CFG,
				    LAN9250_HW_CFG_MBO | LAN9250_HW_CFG_TX_FIF_SZ_8KB);
	if (ret < 0) {
		return ret;
	}

	/* Configure MAC automatic flow control:
	 *
	 *  Reference: Microchip Ethernet LAN9250
	 *  https://github.com/microchip-pic-avr-solutions/ethernet-lan9250/
	 *  LAN_Regwrite32(AFC_CFG, 0x006E3741);
	 *
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_AFC_CFG, 0x006e3741);
	if (ret < 0) {
		return ret;
	}

	/* Configure interrupt:
	 *
	 *   - Interrupt De-assertion interval: 100
	 *   - Interrupt output to pin
	 *   - Interrupt pin active output low
	 *   - Interrupt pin push-pull driver
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_IRQ_CFG,
				    LAN9250_IRQ_CFG_INT_DEAS_100US | LAN9250_IRQ_CFG_IRQ_EN |
					    LAN9250_IRQ_CFG_IRQ_TYPE_PP);
	if (ret < 0) {
		return ret;
	}

	/* Configure interrupt trigger source, please refer to macro
	 * LAN9250_INT_SOURCE.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_INT_EN,
				    LAN9250_INT_EN_PHY_INT_EN | LAN9250_INT_EN_RSFL_EN);
	if (ret < 0) {
		return ret;
	}

	/* Disable TX data FIFO available interrupt */
	ret = lan9250_write_sys_reg(dev, LAN9250_FIFO_INT,
				    LAN9250_FIFO_INT_TX_DATA_AVAILABLE_LEVEL |
					    LAN9250_FIFO_INT_TX_STATUS_LEVEL);
	if (ret < 0) {
		return ret;
	}

	/* Configure RX:
	 *
	 *   - RX DMA counter: Ethernet maximum packet size
	 *   - RX data offset: 4, so that need read dummy before reading data
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_RX_CFG, 0x06000000 | 0x00000400);
	if (ret < 0) {
		return ret;
	}

	/* Configure remote power management:
	 *
	 *   - Auto wakeup
	 *   - Disable 1588 clock, unless the PTP clock is used
	 *   - Disable 1588 timestamp unit clock, unless the PTP clock is used
	 *   - Energy-detect
	 *   - Wake on
	 *   - Clear wakeon
	 *
	 * The 1588 clock disable bits are only set when written as 1 two
	 * consecutive times.
	 */
	for (int i = 0; i < 2; i++) {
		ret = lan9250_write_sys_reg(
			dev, LAN9250_PMT_CTRL,
			LAN9250_PMT_CTRL_PM_WAKE | LAN9250_PMT_CTRL_WOL_EN |
				LAN9250_PMT_CTRL_WOL_STS |
				(IS_ENABLED(CONFIG_PTP_CLOCK_LAN9250)
					 ? 0
					 : (LAN9250_PMT_CTRL_1588_DIS |
					    LAN9250_PMT_CTRL_1588_TSU_DIS)));
		if (ret < 0) {
			return ret;
		}
	}

	/* Configure PHY auto-negotiation advertisement capability:
	 *
	 *   - Asymmetric pause
	 *   - Symmetric pause
	 *   - 100Base-X half/full duplex
	 *   - 10Base-X half/full duplex
	 *   - Select IEEE802.3
	 */
	ret = lan9250_write_phy_reg(
		dev, LAN9250_PHY_AN_ADV,
		LAN9250_PHY_AN_ADV_ASYM_PAUSE | LAN9250_PHY_AN_ADV_SYM_PAUSE |
			LAN9250_PHY_AN_ADV_100BTX_HD | LAN9250_PHY_AN_ADV_100BTX_FD |
			LAN9250_PHY_AN_ADV_10BT_HD | LAN9250_PHY_AN_ADV_10BT_FD |
			LAN9250_PHY_AN_ADV_SELECTOR_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY basic control:
	 *
	 *   - Auto-Negotiation for 10/100 Mbits and Half/Full Duplex
	 *   - Restart Auto-Negotiation to apply the advertisement
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_BASIC_CONTROL,
				    LAN9250_PHY_BASIC_CONTROL_PHY_AN |
					    LAN9250_PHY_BASIC_CONTROL_PHY_RST_AN |
					    LAN9250_PHY_BASIC_CONTROL_PHY_SPEED_SEL_LSB |
					    LAN9250_PHY_BASIC_CONTROL_PHY_DUPLEX);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY special mode:
	 *
	 *   - PHY mode = 111b, enable all capable and auto-nagotiation
	 *   - PHY address = 1, default value is fixed to 1 by manufacturer
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_SPECIAL_MODES, 0x00E0 | 1);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY special control or status indication:
	 *
	 *   - Port auto-MDIX determined by bits 14 and 13
	 *   - Auto-MDIX
	 *   - Disable SQE tests
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STAT_IND,
				    LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXCTRL |
					    LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXEN |
					    LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_SQEOFF);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY interrupt source:
	 *
	 *   - Link up
	 *   - Link down
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_INTERRUPT_MASK,
				    LAN9250_PHY_INTERRUPT_MASK_LINK_UP |
					    LAN9250_PHY_INTERRUPT_MASK_LINK_DOWN);
	if (ret < 0) {
		return ret;
	}

	/* Configure special control or status:
	 *
	 *   - Fixed to write 0000010b to reserved filed
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STATUS,
				    LAN9250_PHY_MODE_CONTROL_STATUS_ALTINT);
	if (ret < 0) {
		return ret;
	}

	/* Clear interrupt status */
	ret = lan9250_write_sys_reg(dev, LAN9250_INT_STS, 0xFFFFFFFFU);
	if (ret < 0) {
		return ret;
	}

	/* Configure HMAC control:
	 *
	 *   - No pad stripping, so that every RX frame includes the CRC
	 *   - Full duplex, updated from the PHY on link up
	 *   - TX enable
	 *   - RX enable
	 *   - Pass all multicast frames
	 *   - Hash filtering disabled
	 *   - Promiscuous disabled
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_CR,
				    LAN9250_HMAC_CR_TXEN | LAN9250_HMAC_CR_RXEN |
					    LAN9250_HMAC_CR_FDPX | LAN9250_HMAC_CR_MCPAS);
	if (ret < 0) {
		return ret;
	}

	/* Configure HMAC flow control:
	 *
	 *   - Pause time sent in automatic pause frames: maximum. Pause frames
	 *     with a pause time of zero are sent when the RX FIFO drains, see
	 *     AFC_CFG.
	 *   - Act on received pause frames (full duplex) and enable
	 *     backpressure (half duplex), as pause is advertised by the PHY
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_FLOW,
				    LAN9250_HMAC_FLOW_FCPT_MAX | LAN9250_HMAC_FLOW_FCEN);
	if (ret < 0) {
		return ret;
	}

	/* Configure HMAC VLAN:
	 *
	 * If used, this register is typically set to the standard VLAN value of
	 * 8100h. If both VLAN1 and VLAN2 set to the same value, VLAN1 is
	 * given higher precedence and the maximum legal frame length is
	 * set to 1522.
	 */
#if defined(CONFIG_NET_VLAN)
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_VLAN1, NET_ETH_PTYPE_VLAN);
	if (ret < 0) {
		return ret;
	}
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_VLAN2, NET_ETH_PTYPE_VLAN);
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_PTP_CLOCK_LAN9250)
	ret = lan9250_1588_init(dev);
	if (ret < 0) {
		return ret;
	}
#endif

	/* Configure TX:
	 *
	 *   - TX enable
	 */
	return lan9250_write_sys_reg(dev, LAN9250_TX_CFG, LAN9250_TX_CFG_TX_ON);
}

static int lan9250_write_buf(const struct device *dev, uint8_t *data_buffer, uint16_t buf_len)
{
	const struct lan9250_config *config = dev->config;
	uint8_t cmd[1] = {LAN9250_SPI_INSTR_WRITE};
	uint8_t instr[2] = {(LAN9250_TX_DATA_FIFO >> 8) & 0xFF, (LAN9250_TX_DATA_FIFO & 0xFF)};
	struct spi_buf tx_buf[3];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 3};

	tx_buf[0].buf = &cmd;
	tx_buf[0].len = ARRAY_SIZE(cmd);
	tx_buf[1].buf = &instr;
	tx_buf[1].len = ARRAY_SIZE(instr);
	tx_buf[2].buf = data_buffer;
	tx_buf[2].len = buf_len;

	return spi_transceive_dt(&config->spi, &tx, NULL);
}

static int lan9250_read_buf(const struct device *dev, uint8_t *data_buffer, uint16_t buf_len)
{
	return lan9250_read(dev, LAN9250_RX_DATA_FIFO, data_buffer, buf_len);
}

/* Discard the current frame from the RX data FIFO. Its status has already
 * been read from the RX status FIFO.
 */
static int lan9250_rx_discard(const struct device *dev, uint16_t pkt_len)
{
	/* RX data offset and frame data, padded to a DWORD */
	uint16_t dwords = (LAN9250_RX_DATA_OFFSET + pkt_len + 3) / 4;
	uint32_t tmp;
	int ret;

	/* Fast-forward needs at least 4 DWORDs of frame data in the FIFO */
	if (((pkt_len + 3) / 4) < 4) {
		for (uint16_t i = 0; i < dwords; i++) {
			ret = lan9250_read_sys_reg(dev, LAN9250_RX_DATA_FIFO, &tmp);
			if (ret < 0) {
				return ret;
			}
		}

		return 0;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_RX_DP_CTRL, LAN9250_RX_DP_CTRL_RX_FFWD);
	if (ret < 0) {
		return ret;
	}

	return lan9250_wait_ready(dev, LAN9250_RX_DP_CTRL, LAN9250_RX_DP_CTRL_RX_FFWD, 0,
				  LAN9250_MAC_TIMEOUT);
}

static int lan9250_rx_frame(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;
	struct net_pkt *pkt;
	uint16_t pkt_len;
	uint32_t tmp;
	int ret;

	/* Check packet status, the length includes the CRC */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_STATUS_FIFO, &tmp);
	if (ret < 0) {
		return ret;
	}
	pkt_len = (tmp & LAN9250_RX_STS_PACKET_LEN) >> 16;

	if (((tmp & LAN9250_RX_STS_ES) != 0) || (pkt_len < LAN9250_RX_MIN_LEN) ||
	    (pkt_len > LAN9250_RX_MAX_LEN)) {
		LOG_DBG("Dropping RX frame, status 0x%08x", tmp);
		eth_stats_update_errors_rx(ctx->iface);
		return lan9250_rx_discard(dev, pkt_len);
	}

	/* Read dummy  data */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_DATA_FIFO, &tmp);
	if (ret < 0) {
		return ret;
	}

	/* Read the frame including CRC and padding in one DWORD-aligned
	 * transfer, so that the RX data FIFO is at the next frame afterwards.
	 */
	ret = lan9250_read_buf(dev, ctx->buf, LAN9250_ALIGN(pkt_len));
	if (ret < 0) {
		return ret;
	}
	pkt_len -= LAN9250_CRC_LEN;

	pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, pkt_len, NET_AF_UNSPEC, 0,
					   K_MSEC(CONFIG_ETH_LAN9250_BUF_ALLOC_TIMEOUT));
	if (pkt == NULL) {
		LOG_ERR("%s: Could not allocate rx buffer", dev->name);
		eth_stats_update_errors_rx(ctx->iface);
		return 0;
	}

	if (net_pkt_write(pkt, ctx->buf, pkt_len) < 0) {
		LOG_ERR("%s: Could not copy rx frame", dev->name);
		eth_stats_update_errors_rx(ctx->iface);
		net_pkt_unref(pkt);
		return 0;
	}

#if defined(CONFIG_PTP_CLOCK_LAN9250)
	lan9250_1588_rx_ts(dev, pkt, pkt_len);
#endif

	/* Feed buffer frame to IP stack */
	if (net_recv_data(ctx->iface, pkt) < 0) {
		net_pkt_unref(pkt);
	}

	return 0;
}

static int lan9250_rx(const struct device *dev)
{
	uint8_t pktcnt;
	uint32_t tmp;
	int ret;

	/* Check valid packet count */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_FIFO_INF, &tmp);
	if (ret < 0) {
		return ret;
	}
	pktcnt = (tmp & LAN9250_RX_FIFO_INF_RXSUSED) >> 16;

	/* Frames arriving from now on set INT_STS.RSFL again and are handled
	 * on the next interrupt.
	 */
	for (; pktcnt > 0; pktcnt--) {
		ret = lan9250_rx_frame(dev);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int lan9250_tx_frame(const struct device *dev, struct net_pkt *pkt)
{
	struct lan9250_runtime *ctx = dev->data;
	size_t len = net_pkt_get_len(pkt);
	uint32_t regval;
	uint16_t free_size;
	uint8_t status_size;
	k_timepoint_t end;
	uint32_t tmp;
	int ret;

	if (len > NET_ETH_MAX_FRAME_SIZE) {
		LOG_ERR("TX frame too long: %zu", len);
		return -EMSGSIZE;
	}

	/* Wait for room for TX commands 'A' and 'B' and the padded frame */
	end = sys_timepoint_calc(K_MSEC(LAN9250_TX_TIMEOUT));
	while (true) {
		ret = lan9250_read_sys_reg(dev, LAN9250_TX_FIFO_INF, &regval);
		if (ret < 0) {
			return ret;
		}

		free_size = regval & LAN9250_TX_FIFO_INF_TXFREE;
		if (free_size >= LAN9250_ALIGN(len) + 2 * sizeof(uint32_t)) {
			break;
		}

		if (sys_timepoint_expired(end)) {
			LOG_ERR("TX FIFO full");
			eth_stats_update_errors_tx(ctx->iface);
			return -EBUSY;
		}

		k_msleep(1);
	}

	status_size = (regval & LAN9250_TX_FIFO_INF_TXSUSED) >> 16;

	/* TX command 'A' */
	ret = lan9250_write_sys_reg(
		dev, LAN9250_TX_DATA_FIFO,
		LAN9250_TX_CMD_A_INT_ON_COMP | LAN9250_TX_CMD_A_BUFFER_ALIGN_4B |
			LAN9250_TX_CMD_A_START_OFFSET_0B | LAN9250_TX_CMD_A_FIRST_SEG |
			LAN9250_TX_CMD_A_LAST_SEG | len);
	if (ret < 0) {
		return ret;
	}

	/* TX command 'B' */
	ret = lan9250_write_sys_reg(dev, LAN9250_TX_DATA_FIFO, LAN9250_TX_CMD_B_PACKET_TAG | len);
	if (ret < 0) {
		return ret;
	}

	if (net_pkt_read(pkt, ctx->buf, len)) {
		return -EIO;
	}

	ret = lan9250_write_buf(dev, ctx->buf, LAN9250_ALIGN(len));
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < status_size; i++) {
		ret = lan9250_read_sys_reg(dev, LAN9250_TX_STATUS_FIFO, &tmp);
		if (ret < 0) {
			return ret;
		}
	}

#if defined(CONFIG_PTP_CLOCK_LAN9250)
	if (net_pkt_is_tx_timestamping(pkt)) {
		lan9250_1588_tx_ts(dev, pkt, len);
	}
#endif

	return 0;
}

static int lan9250_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct lan9250_runtime *ctx = dev->data;
	int ret;

	k_mutex_lock(&ctx->lock, K_FOREVER);
	ret = lan9250_tx_frame(dev, pkt);
	k_mutex_unlock(&ctx->lock);

	return ret;
}

static void lan9250_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct lan9250_runtime *context = CONTAINER_OF(cb, struct lan9250_runtime, gpio_cb);

	k_sem_give(&context->int_sem);
}

static int lan9250_update_duplex(const struct device *dev)
{
	uint16_t phy_sts;
	uint32_t mac_cr;
	uint32_t new_cr;
	int ret;

	ret = lan9250_read_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STATUS, &phy_sts);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_read_mac_reg(dev, LAN9250_HMAC_CR, &mac_cr);
	if (ret < 0) {
		return ret;
	}

	if ((phy_sts & LAN9250_PHY_SPECIAL_CONTROL_STATUS_SPEED_FDPX) != 0) {
		new_cr = mac_cr | LAN9250_HMAC_CR_FDPX;
	} else {
		new_cr = mac_cr & ~LAN9250_HMAC_CR_FDPX;
	}

	LOG_DBG("Link %s duplex", (new_cr & LAN9250_HMAC_CR_FDPX) != 0 ? "full" : "half");

	if (new_cr == mac_cr) {
		return 0;
	}

	return lan9250_write_mac_reg(dev, LAN9250_HMAC_CR, new_cr);
}

static int lan9250_handle_link(const struct device *dev)
{
	struct lan9250_runtime *context = dev->data;
	uint16_t tmp;
	int ret;

	/* Read PHY interrupt source register */
	ret = lan9250_read_phy_reg(dev, LAN9250_PHY_INTERRUPT_SOURCE, &tmp);
	if (ret < 0) {
		return ret;
	}

	if ((tmp & (LAN9250_PHY_INTERRUPT_SOURCE_LINK_UP |
		    LAN9250_PHY_INTERRUPT_SOURCE_LINK_DOWN)) == 0) {
		return 0;
	}

	/* Link up and down may both be latched after a short link drop, so
	 * report the current link status. The link status bit latches low,
	 * read it twice.
	 */
	ret = lan9250_read_phy_reg(dev, LAN9250_PHY_BASIC_STATUS, &tmp);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_read_phy_reg(dev, LAN9250_PHY_BASIC_STATUS, &tmp);
	if (ret < 0) {
		return ret;
	}

	if ((tmp & LAN9250_PHY_BASIC_STATUS_LINK_STATUS) != 0) {
		/* Match the MAC duplex mode to the negotiated one */
		ret = lan9250_update_duplex(dev);
		if (ret < 0) {
			return ret;
		}

		net_eth_carrier_on(context->iface);
	} else {
		net_eth_carrier_off(context->iface);
	}

	return 0;
}

static int lan9250_handle_irq(const struct device *dev)
{
	uint32_t int_sts;
	uint32_t ier;
	int ret;

	/* Save interrupt enable register value */
	ret = lan9250_read_sys_reg(dev, LAN9250_INT_EN, &ier);
	if (ret < 0) {
		return ret;
	}

	/* Disable interrupts to release the interrupt line */
	ret = lan9250_write_sys_reg(dev, LAN9250_INT_EN, 0);
	if (ret < 0) {
		return ret;
	}

	/* Read interrupt status register */
	ret = lan9250_read_sys_reg(dev, LAN9250_INT_STS, &int_sts);
	if (ret < 0) {
		goto reenable;
	}

	if ((int_sts & LAN9250_INT_STS_PHY_INT) != 0) {
		ret = lan9250_handle_link(dev);
		if (ret < 0) {
			LOG_ERR("PHY interrupt handling failed: %d", ret);
		}
	}

	if ((int_sts & LAN9250_INT_STS_RSFL) != 0) {
		ret = lan9250_write_sys_reg(dev, LAN9250_INT_STS, LAN9250_INT_STS_RSFL);
		if (ret == 0) {
			ret = lan9250_rx(dev);
		}
		if (ret < 0) {
			LOG_ERR("RX failed: %d", ret);
		}
	}

reenable:
	/* Re-enable interrupts */
	return lan9250_write_sys_reg(dev, LAN9250_INT_EN, ier);
}

static void lan9250_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct lan9250_runtime *context = dev->data;
	int ret;

	while (true) {
		k_sem_take(&context->int_sem, K_FOREVER);

		k_mutex_lock(&context->lock, K_FOREVER);
		ret = lan9250_handle_irq(dev);
		k_mutex_unlock(&context->lock);

		if (ret < 0) {
			LOG_ERR("Interrupt handling failed: %d", ret);
		}
	}
}

static enum ethernet_hw_caps lan9250_get_capabilities(const struct device *dev __unused,
						      struct net_if *iface __unused)
{
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
		| ETHERNET_PROMISC_MODE
#endif
#if defined(CONFIG_NET_VLAN)
		| ETHERNET_HW_VLAN
#endif
	;
}

static void lan9250_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct lan9250_runtime *context = dev->data;

	net_if_set_link_addr(iface, context->mac_address, sizeof(context->mac_address),
			     NET_LINK_ETHERNET);
	context->iface = iface;
	ethernet_init(iface);

	net_if_carrier_off(iface);

	k_thread_create(&context->thread, context->thread_stack,
			CONFIG_ETH_LAN9250_RX_THREAD_STACK_SIZE,
			lan9250_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_LAN9250_RX_THREAD_PRIO), 0, K_NO_WAIT);
}

static int lan9250_set_promisc(const struct device *dev, bool enable)
{
	uint32_t reg;
	int ret;

	ret = lan9250_read_mac_reg(dev, LAN9250_HMAC_CR, &reg);
	if (ret < 0) {
		return ret;
	}

	/* See Table 11-1 from the LAN9250 data sheet */
	if (enable) {
		if ((reg & LAN9250_HMAC_CR_PRMS) != 0) {
			return -EALREADY;
		}

		reg &= ~LAN9250_HMAC_CR_MCPAS;
		reg |= LAN9250_HMAC_CR_PRMS;
		reg &= ~LAN9250_HMAC_CR_HO;
	} else {
		if ((reg & LAN9250_HMAC_CR_PRMS) == 0) {
			return -EALREADY;
		}

		reg |= LAN9250_HMAC_CR_MCPAS;
		reg &= ~LAN9250_HMAC_CR_PRMS;
		reg &= ~LAN9250_HMAC_CR_HO;
	}

	return lan9250_write_mac_reg(dev, LAN9250_HMAC_CR, reg);
}

static int lan9250_set_config(const struct device *dev,
			      struct net_if *iface __unused,
			      enum ethernet_config_type type,
			      const struct ethernet_config *config)
{
	struct lan9250_runtime *ctx = dev->data;
	int ret;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		k_mutex_lock(&ctx->lock, K_FOREVER);
		memcpy(ctx->mac_address, config->mac_address.addr,
		       sizeof(ctx->mac_address));
		ret = lan9250_set_macaddr(dev);
		k_mutex_unlock(&ctx->lock);
		if (ret < 0) {
			LOG_ERR("Set mac address failed");
			return ret;
		}

		LOG_INF("%s MAC set to %02x:%02x:%02x:%02x:%02x:%02x",
			dev->name,
			ctx->mac_address[0], ctx->mac_address[1],
			ctx->mac_address[2], ctx->mac_address[3],
			ctx->mac_address[4], ctx->mac_address[5]);

		return 0;
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		if (IS_ENABLED(CONFIG_NET_PROMISCUOUS_MODE)) {
			k_mutex_lock(&ctx->lock, K_FOREVER);
			ret = lan9250_set_promisc(dev, config->promisc_mode);
			k_mutex_unlock(&ctx->lock);

			return ret;
		}

		break;
	default:
		break;
	}

	return -ENOTSUP;
}

#if defined(CONFIG_PTP_CLOCK_LAN9250)
static const struct device *lan9250_get_ptp_clock(const struct device *dev,
						  struct net_if *iface __unused)
{
	const struct lan9250_config *config = dev->config;

	return config->ptp_clock;
}

static int lan9250_ptp_clock_set(const struct device *dev, struct net_ptp_time *tm)
{
	const struct device *eth_dev = dev->config;
	struct lan9250_runtime *ctx = eth_dev->data;
	int ret;

	/* The 1588 clock has 32 bits of seconds */
	if ((tm->second > UINT32_MAX) || (tm->nanosecond >= NSEC_PER_SEC)) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);

	ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CLOCK_SEC, (uint32_t)tm->second);
	if (ret < 0) {
		goto out;
	}

	ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CLOCK_NS, tm->nanosecond);
	if (ret < 0) {
		goto out;
	}

	ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CLOCK_SUBNS, 0);
	if (ret < 0) {
		goto out;
	}

	ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CMD_CTL,
				    LAN9250_1588_CMD_CTL_CLOCK_LOAD);

out:
	k_mutex_unlock(&ctx->lock);

	return ret;
}

static int lan9250_ptp_clock_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct device *eth_dev = dev->config;
	struct lan9250_runtime *ctx = eth_dev->data;
	/* Clock seconds and clock nanoseconds */
	uint8_t data[2 * sizeof(uint32_t)];
	int ret;

	k_mutex_lock(&ctx->lock, K_FOREVER);

	/* Save the current clock value into the clock registers */
	ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CMD_CTL,
				    LAN9250_1588_CMD_CTL_CLOCK_READ);
	if (ret == 0) {
		ret = lan9250_read(eth_dev, LAN9250_SPI_ADDR_INC | LAN9250_1588_CLOCK_SEC, data,
				   sizeof(data));
	}

	k_mutex_unlock(&ctx->lock);

	if (ret < 0) {
		return ret;
	}

	tm->second = sys_get_le32(&data[0]);
	tm->nanosecond = sys_get_le32(&data[4]) & LAN9250_1588_NS_MASK;

	return 0;
}

static int lan9250_ptp_clock_adjust(const struct device *dev, int increment)
{
	const struct device *eth_dev = dev->config;
	struct lan9250_runtime *ctx = eth_dev->data;
	int32_t sec = increment / (int32_t)NSEC_PER_SEC;
	int32_t nsec = increment % (int32_t)NSEC_PER_SEC;
	int ret = 0;

	/* Only positive nanosecond steps are supported, so step the seconds
	 * down and the nanoseconds up for negative adjustments.
	 */
	if (nsec < 0) {
		nsec += NSEC_PER_SEC;
		sec--;
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);

	if (sec != 0) {
		ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CLOCK_STEP_ADJ,
					    (sec > 0 ? LAN9250_1588_CLOCK_STEP_ADJ_ADD : 0) |
						    (uint32_t)abs(sec));
		if (ret < 0) {
			goto out;
		}

		ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CMD_CTL,
					    LAN9250_1588_CMD_CTL_CLOCK_STEP_SEC);
		if (ret < 0) {
			goto out;
		}
	}

	if (nsec != 0) {
		/* The step replaces the regular increment of one clock period */
		ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CLOCK_STEP_ADJ,
					    LAN9250_1588_CLOCK_STEP_ADJ_ADD |
						    (nsec + LAN9250_1588_CLOCK_PERIOD_NS));
		if (ret < 0) {
			goto out;
		}

		ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CMD_CTL,
					    LAN9250_1588_CMD_CTL_CLOCK_STEP_NS);
	}

out:
	k_mutex_unlock(&ctx->lock);

	return ret;
}

static int lan9250_ptp_clock_rate_adjust(const struct device *dev, double ratio)
{
	const struct device *eth_dev = dev->config;
	struct lan9250_runtime *ctx = eth_dev->data;
	/* Rate adjustment in 2^-32 ns per clock period */
	double adj = (ratio - 1.0) * LAN9250_1588_CLOCK_PERIOD_NS * (double)BIT64(32);
	uint32_t val;
	int ret;

	if ((adj > LAN9250_1588_CLOCK_RATE_ADJ_VALUE) ||
	    (adj < -(double)LAN9250_1588_CLOCK_RATE_ADJ_VALUE)) {
		return -EINVAL;
	}

	if (adj >= 0) {
		val = LAN9250_1588_CLOCK_RATE_ADJ_FASTER | (uint32_t)(adj + 0.5);
	} else {
		val = (uint32_t)(-adj + 0.5);
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);
	ret = lan9250_write_sys_reg(eth_dev, LAN9250_1588_CLOCK_RATE_ADJ, val);
	k_mutex_unlock(&ctx->lock);

	return ret;
}

static DEVICE_API(ptp_clock, lan9250_ptp_api) = {
	.set = lan9250_ptp_clock_set,
	.get = lan9250_ptp_clock_get,
	.adjust = lan9250_ptp_clock_adjust,
	.rate_adjust = lan9250_ptp_clock_rate_adjust,
};
#endif /* CONFIG_PTP_CLOCK_LAN9250 */

static const struct ethernet_api api_funcs = {
	.iface_api.init = lan9250_iface_init,
	.get_capabilities = lan9250_get_capabilities,
	.set_config = lan9250_set_config,
	.send = lan9250_tx,
#if defined(CONFIG_PTP_CLOCK_LAN9250)
	.get_ptp_clock = lan9250_get_ptp_clock,
#endif
};

static int lan9250_init(const struct device *dev)
{
	int ret;
	const struct lan9250_config *config = dev->config;
	struct lan9250_runtime *context = dev->data;

	/* SPI config */
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI master port %s not ready", config->spi.bus->name);
		return -EINVAL;
	}

	/* Initialize GPIO */
	if (!gpio_is_ready_dt(&config->interrupt)) {
		LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
		return ret;
	}

	gpio_init_callback(&(context->gpio_cb), lan9250_gpio_callback,
			   BIT(config->interrupt.pin));
	ret = gpio_add_callback(config->interrupt.port, &context->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Unable to add GPIO callback %u", config->interrupt.pin);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Unable to enable GPIO INT %u", config->interrupt.pin);
		return ret;
	}

	if (config->reset.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset)) {
			LOG_ERR("GPIO port %s not ready", config->reset.port->name);
			return -EINVAL;
		}

		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Unable to configure GPIO pin %u", config->reset.pin);
			return ret;
		}

		/* See Section 19.6.3 from the LAN9250 Data Sheet
		 *
		 * trstia is 200 microseconds min (use 250 us)
		 * tcfg is 15 milliseconds min (use 20 ms for after reset)
		 */
		gpio_pin_set_dt(&config->reset, 1);
		k_usleep(250);
		gpio_pin_set_dt(&config->reset, 0);
		k_msleep(20);
	}

	/* Reset and wait for ready on the LAN9250 SPI device */
	ret = lan9250_sw_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed");
		return ret;
	}
	ret = lan9250_configure(dev);
	if (ret < 0) {
		LOG_ERR("Configuration failed");
		return ret;
	}

	(void)net_eth_mac_load(&config->mac_cfg, context->mac_address);
	ret = lan9250_set_macaddr(dev);
	if (ret < 0) {
		LOG_ERR("Set mac address failed");
		return ret;
	}

	LOG_INF("LAN9250 Initialized");

	return 0;
}

#define LAN9250_DEFINE(inst)                                                                       \
	static struct lan9250_runtime lan9250_##inst##_runtime = {                                 \
		.lock = Z_MUTEX_INITIALIZER(lan9250_##inst##_runtime.lock),                        \
		.int_sem = Z_SEM_INITIALIZER(lan9250_##inst##_runtime.int_sem, 0, UINT_MAX),       \
	};                                                                                         \
                                                                                                   \
	IF_ENABLED(CONFIG_PTP_CLOCK_LAN9250, (DEVICE_DECLARE(lan9250_ptp_clock_##inst);))          \
                                                                                                   \
	static const struct lan9250_config lan9250_##inst##_config = {                             \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8)),                                \
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                               \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(inst),                                  \
		IF_ENABLED(CONFIG_PTP_CLOCK_LAN9250,                                               \
			   (.ptp_clock = DEVICE_GET(lan9250_ptp_clock_##inst),))                   \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, lan9250_init, NULL, &lan9250_##inst##_runtime,         \
				      &lan9250_##inst##_config, CONFIG_ETH_INIT_PRIORITY,          \
				      &api_funcs, NET_ETH_MTU);                                    \
                                                                                                   \
	IF_ENABLED(CONFIG_PTP_CLOCK_LAN9250,                                                       \
		   (DEVICE_DEFINE(lan9250_ptp_clock_##inst, PTP_CLOCK_NAME, NULL, NULL, NULL,      \
				  DEVICE_DT_INST_GET(inst), POST_KERNEL,                           \
				  CONFIG_ETH_INIT_PRIORITY, &lan9250_ptp_api);))
DT_INST_FOREACH_STATUS_OKAY(LAN9250_DEFINE);
