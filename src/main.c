#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bridge.h"

LOG_MODULE_REGISTER(bridge, LOG_LEVEL_INF);

/* One pool and queue per direction, see bridge.h. */
K_MEM_SLAB_DEFINE(bridge_frame_slab_dect_tx, sizeof(struct bridge_frame), FRAME_POOL_COUNT, 4);
K_MEM_SLAB_DEFINE(bridge_frame_slab_spi_tx, sizeof(struct bridge_frame), FRAME_POOL_COUNT, 4);
K_MSGQ_DEFINE(bridge_dect_tx_q, sizeof(struct bridge_frame *), FRAME_POOL_COUNT, 4);
K_MSGQ_DEFINE(bridge_spi_tx_q, sizeof(struct bridge_frame *), FRAME_POOL_COUNT, 4);

/* Stats reporter. Runs at the lowest application priority so the log
 * packaging can never delay a time-critical path; main() is not used for this
 * because it runs at priority 0, tied with dect.c's tx_tid.
 */
static void report_thread(void)
{
	while (1)
	{
 		k_sleep(K_SECONDS(5));
/*		LOG_INF("-----");
		dect_log_stats();
		LOG_INF("-----");
		spi_bridge_log_stats(); */
	}
}

K_THREAD_DEFINE(report_tid, 2048, report_thread, NULL, NULL, NULL,
				K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

int main(void)
{
	int err;

	LOG_INF("bridge starting: 2x %u frames of %u B (%u B total pool)",
			FRAME_POOL_COUNT, (unsigned)sizeof(struct bridge_frame),
			(unsigned)(2 * FRAME_POOL_COUNT * sizeof(struct bridge_frame)));

	/* SPI first: it configures the handshake GPIOs its thread waits for. */
	err = spi_bridge_init();
	if (err)
	{
		LOG_ERR("SPI init failed, err %d", err);
		return err;
	}

	err = dect_init();
	if (err)
	{
		LOG_ERR("DECT init failed, err %d", err);
		return err;
	}

	/* Everything runs in its own threads from here on. */
	return 0;
}
