#ifndef BRIDGE_H
#define BRIDGE_H

/*
 * bridge.h -- internal glue between spi.c, dect.c and main.c: the buffer
 * layout, the pools, the two queues and each module's entry points.
 * protocol.h is the wire contract shared with the Teensy boards.
 *
 * Data flow, one direction (both boards run both):
 *
 *   Teensy A --SPI--> spi.c --bridge_dect_tx_q--> dect.c --air-->
 *   --air--> dect.c --bridge_spi_tx_q--> spi.c --SPI--> Teensy B
 */

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>

#include "protocol.h"

/* Largest Ethernet frame the Teensy can hand us (802.1Q tagged, no FCS). */
#define ETH_MAX_PAYLOAD 1522

/* Frames in flight anywhere in the bridge. */
#define FRAME_POOL_COUNT 16

/*
 * The one buffer layout used everywhere.
 *
 *   offset  0 : nrf_tx_rx_header                       30 B
 *               +0  union { cycle, tx_start_lo }             <- written by dect.c on TX
 *                         { dect_rx_ns, nrf_spi_tx_ns }      <- dect_rx_ns set on RX
 *               +8  nrf_spi_rx_ns
 *               +12 dect_tx_ns
 *               +16 t_rx                               14 B  <- SPI RX lands here
 *   offset 30 : eth[]                                        <- payload continues here
 *
 * Because t_rx is the last member of nrf_tx_rx_header, one SPI read of
 * sizeof(t_rx_header) + eth_len bytes starting at &frame->hdr.t_rx fills the
 * Teensy's header and the Ethernet payload in a single transfer, with the
 * locally owned timestamp words already in front of it. On the far side the
 * same buffer goes out over SPI verbatim from offset 0. The asserts below keep
 * this layout safe against edits to protocol.h.
 */
struct bridge_frame
{
	nrf_tx_rx_header hdr;
	uint8_t eth[ETH_MAX_PAYLOAD];
} __packed;

BUILD_ASSERT(sizeof(t_rx_header) == 14, "t_rx_header changed size");
BUILD_ASSERT(sizeof(nrf_tx_rx_header) == 30, "nrf_tx_rx_header changed size");
BUILD_ASSERT(sizeof(t_tx_header) == 38, "t_tx_header changed size");
BUILD_ASSERT(offsetof(nrf_tx_rx_header, t_rx) + sizeof(t_rx_header) ==
				 sizeof(nrf_tx_rx_header),
			 "t_rx must stay last in nrf_tx_rx_header: the SPI RX writes in place");
BUILD_ASSERT(offsetof(struct bridge_frame, eth) == sizeof(nrf_tx_rx_header),
			 "eth[] must follow t_rx with no padding");
BUILD_ASSERT(sizeof(struct bridge_frame) % 4 == 0,
			 "slab block size must be a multiple of the slab alignment");

/* Bytes on the SPI wire in one data phase, per direction, excluding payload. */
#define SPI_RX_HDR_BYTES ((uint16_t)sizeof(t_rx_header))	  /* Teensy A -> nRF */
#define SPI_TX_HDR_BYTES ((uint16_t)sizeof(nrf_tx_rx_header)) /* nRF -> Teensy B */

/* Defined in main.c. One pool per direction rather than a shared one, so
 * frames stuck in one queue (e.g. bridge_spi_tx_q behind a slow Teensy) can
 * never starve the other direction's allocations.
 */
extern struct k_mem_slab bridge_frame_slab_dect_tx; /* feeds bridge_dect_tx_q */
extern struct k_mem_slab bridge_frame_slab_spi_tx;	/* feeds bridge_spi_tx_q */
extern struct k_msgq bridge_dect_tx_q;				/* SPI RX  -> DECT TX */
extern struct k_msgq bridge_spi_tx_q;				/* DECT RX -> SPI TX */

static inline int bridge_dect_tx_frame_alloc(struct bridge_frame **frame, k_timeout_t timeout)
{
	return k_mem_slab_alloc(&bridge_frame_slab_dect_tx, (void **)frame, timeout);
}

static inline void bridge_dect_tx_frame_free(struct bridge_frame *frame)
{
	k_mem_slab_free(&bridge_frame_slab_dect_tx, (void *)frame);
}

static inline int bridge_spi_tx_frame_alloc(struct bridge_frame **frame, k_timeout_t timeout)
{
	return k_mem_slab_alloc(&bridge_frame_slab_spi_tx, (void **)frame, timeout);
}

static inline void bridge_spi_tx_frame_free(struct bridge_frame *frame)
{
	k_mem_slab_free(&bridge_frame_slab_spi_tx, (void *)frame);
}

/* spi.c */
int spi_bridge_init(void);
void spi_bridge_tx_notify(void); /* a frame is waiting: ask the Teensy for clocks */
void spi_bridge_log_stats(void);

/* dect.c */
int dect_init(void);
void dect_log_stats(void);
/* Converts a k_uptime_ticks() sample into the "ns since PPS edge" frame that
 * dect_tx_ns / dect_rx_ns are expressed in.
 */
uint32_t dect_uptime_to_ns_since_pps(int64_t uptime_ticks);

#endif /* BRIDGE_H */
