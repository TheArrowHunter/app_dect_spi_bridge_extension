#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_spis.h>

#include "bridge.h"

LOG_MODULE_REGISTER(spi_bridge, LOG_LEVEL_INF);

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi_slave));

/* Same peripheral as spi_dev. spi_xfer() reads SEMSTAT (semaphore ownership),
 * which the Zephyr/nrfx SPIS driver does not expose. It is read-only, so this
 * does not race the driver.
 */
#define SPIS_REG ((NRF_SPIS_Type *)DT_REG_ADDR(DT_NODELABEL(spi_slave)))

static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE,
	.frequency = 0,
	.cs = {.delay = 0},
};

static const struct gpio_dt_spec req_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), spi_req_gpios);
static const struct gpio_dt_spec rdy_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), spi_rdy_gpios);

/* Released by spi_bridge_init() once the GPIOs and SPI device are ready. */
K_SEM_DEFINE(spi_start_sem, 0, 1);

static struct k_poll_signal spi_done_sig;
static struct k_poll_event spi_done_evt;

static struct
{
	atomic_t rx_frames, tx_frames, rx_bytes, tx_bytes;
	atomic_t len_err, xfer_err, nobuf, qfull, hdr_len_mismatch;
} sstats;

/* Pause between arm-failure retries in spi_xfer(). Arm failures are local
 * and invisible to the Teensy, so this only keeps a persistent fault from
 * hot-looping the thread.
 */
#define SPI_ARM_RETRY_MS 50

/* Upper bound on the SEMSTAT spin in spi_xfer(), in ~1 us iterations. The
 * driver's ACQUIRED ISR normally releases the semaphore within a few
 * microseconds; reaching this cap means that ISR is not running.
 */
#define SPI_ARM_SEM_MAX_SPINS 256

/* ------------------------------------------------------------------------- */

/* Arm a transceive, tell the master it may clock, and wait for completion.
 * Either buffer set may be NULL for a one-directional phase. An arm failure
 * is retried in place (the Teensy is still waiting for RDY); an error on
 * completion is logged but not retried, since the Teensy has already run its
 * side of the phase and redoing it could desync the link.
 */
static void spi_xfer(const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	int err;

	for (;;)
	{
		k_poll_signal_reset(&spi_done_sig);
		spi_done_evt.state = K_POLL_STATE_NOT_READY;

		err = spi_transceive_signal(spi_dev, &spi_cfg, tx, rx, &spi_done_sig);
		if (err == 0)
		{
			break;
		}

		atomic_inc(&sstats.xfer_err);
		LOG_ERR("SPI arm failed: %d, retrying in %d ms", err, SPI_ARM_RETRY_MS);
		k_sleep(K_MSEC(SPI_ARM_RETRY_MS));
	}

	/* spi_transceive_signal() returning 0 only means the driver has triggered
	 * the ACQUIRE task. Arming finishes when its ACQUIRED ISR writes the
	 * EasyDMA pointers and triggers RELEASE. A transaction started before that
	 * is ignored (0xff on MISO for the whole frame), so the Teensy would read a
	 * 0xffff length and desync. Hold RDY until the semaphore is no longer the
	 * CPU's (an armed, idle SPIS reads FREE). The spin is capped so a
	 * peripheral whose ISR never runs fails loudly instead of hanging.
	 */
	for (uint32_t spins = 0;
		 nrf_spis_semaphore_status_get(SPIS_REG) == NRF_SPIS_SEMSTAT_CPU;
		 spins++)
	{
		if (spins >= SPI_ARM_SEM_MAX_SPINS)
		{
			atomic_inc(&sstats.xfer_err);
			LOG_ERR("SPIS semaphore still CPU-held ~%u us after arm "
					"(SEMSTAT=%d); raising RDY anyway, the next length "
					"exchange will likely be corrupt",
					spins, (int)nrf_spis_semaphore_status_get(SPIS_REG));
			break;
		}
		k_busy_wait(1);
	}

	gpio_pin_set_dt(&rdy_pin, 1);
	k_poll(&spi_done_evt, 1, K_FOREVER);
	gpio_pin_set_dt(&rdy_pin, 0);

	/* A negative result means the transfer did not happen as asked; it is
	 * not retried, see above.
	 */
	if (spi_done_sig.result < 0)
	{
		atomic_inc(&sstats.xfer_err);
		LOG_ERR("SPI transfer completed with error: %d", spi_done_sig.result);
	}
}

/* A frame has been received into *rx_frame. Hand it on and swap in a fresh
 * buffer. If the pool or the queue is full the frame is dropped and its buffer
 * reused, so the thread always owns a receive buffer and never blocks.
 */
static void handle_rx(struct bridge_frame **rx_frame, uint16_t eth_len, int64_t rx_uptime)
{
	struct bridge_frame *frame = *rx_frame;
	struct bridge_frame *next;

	/* rx_uptime was sampled right after spi_xfer() returned. */
	frame->hdr.nrf_spi_rx_ns = dect_uptime_to_ns_since_pps(rx_uptime);

	if (frame->hdr.t_rx.eth_payload_len != eth_len)
	{
		/* The wire length is what landed in the buffer, so it wins. */
		atomic_inc(&sstats.hdr_len_mismatch);
		frame->hdr.t_rx.eth_payload_len = eth_len;
	}

	atomic_inc(&sstats.rx_frames);
	atomic_add(&sstats.rx_bytes, eth_len);

	if (bridge_dect_tx_frame_alloc(&next, K_NO_WAIT) != 0)
	{
		atomic_inc(&sstats.nobuf);
		return; /* keep the buffer, drop the frame */
	}
	if (k_msgq_put(&bridge_dect_tx_q, &frame, K_NO_WAIT) != 0)
	{
		atomic_inc(&sstats.qfull);
		bridge_dect_tx_frame_free(next);
		return;
	}
	*rx_frame = next;
}

static void spi_slave_thread(void)
{
	struct bridge_frame *rx_frame = NULL;
	struct bridge_frame *tx_frame = NULL;

	uint16_t tx_total = 0;
	uint16_t rx_total = 0;

	struct spi_buf tx_len_buf = {.buf = &tx_total, .len = sizeof(tx_total)};
	struct spi_buf_set tx_len_bufs = {.buffers = &tx_len_buf, .count = 1};
	struct spi_buf rx_len_buf = {.buf = &rx_total, .len = sizeof(rx_total)};
	struct spi_buf_set rx_len_bufs = {.buffers = &rx_len_buf, .count = 1};

	struct spi_buf tx_buf = {0};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {0};
	struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

	k_poll_signal_init(&spi_done_sig);
	k_poll_event_init(&spi_done_evt, K_POLL_TYPE_SIGNAL,
					  K_POLL_MODE_NOTIFY_ONLY, &spi_done_sig);

	k_sem_take(&spi_start_sem, K_FOREVER);

	/* Pre-allocate the first RX buffer. */
	bridge_dect_tx_frame_alloc(&rx_frame, K_FOREVER);

	while (1)
	{
		uint16_t rx_eth = 0;
		bool do_rx = false;

		/* ---- what do we have to send? --------------------------- */
		if (tx_frame == NULL)
		{
			struct bridge_frame *next;

			if (k_msgq_get(&bridge_spi_tx_q, &next, K_NO_WAIT) == 0)
			{
				tx_frame = next;
				gpio_pin_set_dt(&req_pin, 1);
			}
		}

		if (tx_frame != NULL)
		{
			tx_total = SPI_TX_HDR_BYTES +
					   tx_frame->hdr.t_rx.eth_payload_len;
			tx_buf.buf = &tx_frame->hdr;
			tx_buf.len = tx_total;
		}
		else
		{
			tx_total = 0;
		}

		/* ---- phase 1: length exchange --------------------------- */
		rx_total = 0;
		spi_xfer(&tx_len_bufs, &rx_len_bufs);

		/* Nothing on the wire in either direction (the Teensy applies the
		 * same check): retry the length exchange.
		 */
		if (tx_total == 0 && rx_total == 0)
		{
			continue;
		}

		/* ---- phase 2 setup ------------------------------------- */
		if (rx_total != 0)
		{
			if (rx_total < SPI_RX_HDR_BYTES ||
				rx_total > SPI_RX_HDR_BYTES + ETH_MAX_PAYLOAD)
			{
				LOG_WRN("bad RX length %u announced", rx_total);
				atomic_inc(&sstats.len_err);
				/* Phase 2 still runs, but do_rx stays false so the DMA
				 * is not pointed at rx_frame with an untrusted length.
				 */
			}
			else
			{
				rx_eth = rx_total - SPI_RX_HDR_BYTES;

				/* Zero the words this side owns; they are not
				 * on the wire and the slab block is dirty.
				 */
				memset(&rx_frame->hdr, 0,
					   offsetof(nrf_tx_rx_header, t_rx));

				rx_buf.buf = &rx_frame->hdr.t_rx;
				rx_buf.len = rx_total;
				do_rx = true;
			}
		}

		/* ---- phase 2: data exchange ------------------------------ */
		spi_xfer(tx_total ? &tx_bufs : NULL, do_rx ? &rx_bufs : NULL);

		/* Sampled here, not in handle_rx(), so handling time is not
		 * folded into the timestamp.
		 */
		int64_t rx_uptime = k_uptime_ticks();

		if (do_rx)
		{
			handle_rx(&rx_frame, rx_eth, rx_uptime);
		}

		if (tx_frame != NULL)
		{
			atomic_inc(&sstats.tx_frames);
			atomic_add(&sstats.tx_bytes,
					   tx_total - SPI_TX_HDR_BYTES);
			bridge_spi_tx_frame_free(tx_frame);
			tx_frame = NULL;

			/* REQ comes down here only; the dequeue at the top of
			 * the loop re-asserts it if more is waiting.
			 */
			gpio_pin_set_dt(&req_pin, 0);
		}
	}
}

/* Priority 7, the lowest in the app (dect.c: tx_tid 0, scheduler_tid 5, system
 * workqueue 6). DECT TX timing is the tighter constraint, so this thread yields
 * to it. The cost: if this thread is late re-arming, the Teensy's next length
 * field reads as garbage, showing up as "bad RX length" warnings or hdr_mm /
 * len_err growth in spi_bridge_log_stats(). If that happens, raise this
 * priority, at least above the system workqueue.
 */
K_THREAD_DEFINE(spi_tid, 4096, spi_slave_thread, NULL, NULL, NULL, 7, 0, 0);

/* ------------------------------------------------------------------------- */

/* dect.c calls this right after putting a frame on bridge_spi_tx_q, to raise
 * REQ even while the thread is parked in spi_xfer(). The Teensy only clocks
 * in response to REQ or its own data, so without this a frame queued during
 * an idle length exchange would not be announced.
 */
void spi_bridge_tx_notify(void)
{
	gpio_pin_set_dt(&req_pin, 1);
}

int spi_bridge_init(void)
{
	int err;

	if (!device_is_ready(spi_dev))
	{
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&req_pin, GPIO_OUTPUT_INACTIVE);
	if (err)
	{
		LOG_ERR("REQ pin configure failed, err %d", err);
		return err;
	}

	err = gpio_pin_configure_dt(&rdy_pin, GPIO_OUTPUT_INACTIVE);
	if (err)
	{
		LOG_ERR("RDY pin configure failed, err %d", err);
		return err;
	}

	LOG_INF("SPI slave ready: rx hdr %u B, tx hdr %u B, eth max %u B",
			SPI_RX_HDR_BYTES, SPI_TX_HDR_BYTES, ETH_MAX_PAYLOAD);

	k_sem_give(&spi_start_sem);
	return 0;
}

void spi_bridge_log_stats(void)
{
	uint32_t rxb = (uint32_t)atomic_set(&sstats.rx_bytes, 0);
	uint32_t txb = (uint32_t)atomic_set(&sstats.tx_bytes, 0);

	LOG_INF("  spi: rx %ld frm %u B/s | tx %ld frm %u B/s | "
			"len_err %ld xfer_err %ld nobuf %ld qfull %ld hdr_mm %ld",
			atomic_get(&sstats.rx_frames), rxb,
			atomic_get(&sstats.tx_frames), txb,
			atomic_get(&sstats.len_err), atomic_get(&sstats.xfer_err),
			atomic_get(&sstats.nobuf), atomic_get(&sstats.qfull),
			atomic_get(&sstats.hdr_len_mismatch));
}
