/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(net_gptp, CONFIG_NET_GPTP_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/net/net_log.h>
#include <zephyr/precision_timing/precision_clock.h>

#include "gptp_messages.h"
#include "gptp_data_set.h"
#include "gptp_state.h"
#include "gptp_private.h"

/* Offsets above this are corrected with a phase step instead of the rate */
#define GPTP_PORT_CLOCK_STEP_THRESHOLD_NS (50LL * NSEC_PER_MSEC)

static struct precision_pi port_clock_pi[CONFIG_NET_GPTP_NUM_PORTS];
static int port_clock_ref_port;
static k_timepoint_t port_clock_next;

/* The time receiver port clock is synchronized to the grandmaster, so the
 * other port clocks follow it. Without a time receiver port, they follow the
 * first port.
 */
static int gptp_port_clock_ref_port(void)
{
	for (int port = GPTP_PORT_START; port <= GPTP_PORT_END; port++) {
		if (GPTP_GLOBAL_DS()->selected_role[port] == GPTP_PORT_SLAVE) {
			return port;
		}
	}

	return GPTP_PORT_START;
}

/* Measure the offset clk - ref_clk. The reference clock is read before and
 * after the port clock, and the sample with the shortest read window is used.
 */
static int gptp_port_clock_offset(const struct precision_clock *ref_clk,
				  const struct precision_clock *clk, int64_t *offset)
{
	int64_t best_window = INT64_MAX;
	precision_time_t before;
	precision_time_t after;
	precision_time_t time;
	int ret;

	for (int i = 0; i < CONFIG_NET_GPTP_PORT_CLOCK_SYNC_SAMPLES; i++) {
		ret = precision_clock_read(ref_clk, &before);
		if (ret < 0) {
			return ret;
		}

		ret = precision_clock_read(clk, &time);
		if (ret < 0) {
			return ret;
		}

		ret = precision_clock_read(ref_clk, &after);
		if (ret < 0) {
			return ret;
		}

		if (after - before < best_window) {
			best_window = after - before;
			*offset = time - (before + best_window / 2);
		}
	}

	return 0;
}

static void gptp_port_clock_update(int port, const struct precision_clock *ref_clk,
				   const struct precision_clock *clk)
{
	struct precision_pi *pi = &port_clock_pi[GPTP_PORT_INDEX(port)];
	precision_time_t now;
	int64_t scaled_ppm;
	int64_t offset = 0;
	double ppb;
	int ret;

	ret = gptp_port_clock_offset(ref_clk, clk, &offset);
	if (ret < 0) {
		NET_WARN_RATELIMIT("Failed to measure port %d clock offset (%d)", port, ret);
		return;
	}

	if ((offset > GPTP_PORT_CLOCK_STEP_THRESHOLD_NS) ||
	    (offset < -GPTP_PORT_CLOCK_STEP_THRESHOLD_NS)) {
		ret = precision_clock_adjust_phase(clk, -offset);
		if (ret < 0) {
			/* Out of the phase step range of the clock */
			ret = precision_clock_read(ref_clk, &now);
			if (ret == 0) {
				ret = precision_clock_set(clk, now);
			}
		}

		if (ret < 0) {
			NET_WARN_RATELIMIT("Failed to step port %d clock (%d)", port, ret);
			return;
		}

		NET_DBG("Port %d clock stepped by %" PRId64 " ns", port, -offset);
		precision_pi_reset(pi);
		return;
	}

	ppb = precision_pi_update(pi, (double)-offset);

	ret = precision_clock_ppb_to_scaled_ppm(ppb, &scaled_ppm);
	if (ret == 0) {
		ret = precision_clock_adjust_rate(clk, scaled_ppm);
	}

	if (ret < 0) {
		NET_WARN_RATELIMIT("Failed to adjust port %d clock rate (%d)", port, ret);
	}

	if (IS_ENABLED(CONFIG_NET_GPTP_MONITOR_SYNC_STATUS)) {
		NET_INFO("port %d clock offset %9" PRId64 " ns, freq offset %f ppb", port,
			 offset, ppb);
	}
}

void gptp_port_clock_sync(void)
{
	const struct precision_clock_ptp_adapter *ref_adapter;
	const struct precision_clock_ptp_adapter *adapter;
	int ref_port;

	if (!sys_timepoint_expired(port_clock_next)) {
		return;
	}

	port_clock_next = sys_timepoint_calc(K_MSEC(CONFIG_NET_GPTP_PORT_CLOCK_SYNC_INTERVAL));

	ref_port = gptp_port_clock_ref_port();
	if (ref_port != port_clock_ref_port) {
		NET_DBG("Port clocks follow port %d", ref_port);
		port_clock_ref_port = ref_port;

		ARRAY_FOR_EACH_PTR(port_clock_pi, pi) {
			precision_pi_init(pi, (double)CONFIG_PRECISION_TIMING_PI_KP / 1000.0,
					  (double)CONFIG_PRECISION_TIMING_PI_KI / 1000.0);
		}
	}

	ref_adapter = &gptp_clock.clocks[GPTP_PORT_INDEX(ref_port)];
	if (ref_adapter->dev == NULL) {
		return;
	}

	for (int port = GPTP_PORT_START; port <= GPTP_PORT_END; port++) {
		adapter = &gptp_clock.clocks[GPTP_PORT_INDEX(port)];

		/* Ports sharing the reference clock need no synchronization */
		if ((port == ref_port) || (adapter->dev == NULL) ||
		    (adapter->dev == ref_adapter->dev)) {
			continue;
		}

		gptp_port_clock_update(port, precision_clock_ptp_get(ref_adapter),
				       precision_clock_ptp_get(adapter));
	}
}
