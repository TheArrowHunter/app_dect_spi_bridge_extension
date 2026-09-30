#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint16_t eth_payload_len; // Length of raw Ethernet frame to transmit later
    uint32_t delay_budget_ns; // Max allowed bridge transit time
    uint32_t eth_rx_ns;       // Teensy Eth RX PTP Timestamp
    uint32_t t_spi_tx_ns;     // Teensy SPI TX PTP Timestamp
} t_rx_header;

typedef struct __attribute__((packed))
{
    union // 8-byte in-place mutation block
    {
        struct __attribute__((packed))
        {
            uint32_t cycle;       // nrf dect tx
            uint32_t tx_start_lo; // nrf dect tx
        } dect_sched_header;

        struct __attribute__((packed))
        {
            uint32_t dect_rx_ns;    // nrf dect rx Timestamp
            uint32_t nrf_spi_tx_ns; //  nrf spi tx Timestamp
        } nrf_rx_header;
    };

    uint32_t nrf_spi_rx_ns; // nrf spi rx Timestamp
    uint32_t dect_tx_ns;    // nrf dect tx Timestamp

    t_rx_header t_rx; // 14 bytes inherited from Teensy

} nrf_tx_rx_header;

typedef struct __attribute__((packed))
{
    nrf_tx_rx_header nrf_tx_rx; // Contains all 6 previous timestamps
    uint32_t t_spi_rx_ns;       // Teensy B: SPI RX PTP Timestamp
    uint32_t t_eth_tx_ns;       // Teensy B: Eth TX PTP Timestamp
} t_tx_header;

#endif