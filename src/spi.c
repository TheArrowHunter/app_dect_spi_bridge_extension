#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include "spi.h"

// SPI Device Setup
#define SPI_NODE DT_NODELABEL(spi2)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
// Interrupt Data Ready Pin Setup
static const struct gpio_dt_spec drdy_pin = GPIO_DT_SPEC_GET(DT_ALIAS(drdy_pin), gpios);
// SPI Transaction Buffer Size
#define BUF_SIZE 32
// SPI Configuration:
static struct spi_config spi_cfg = {
    .frequency = MHZ(2),
    .operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8),
    .slave = 0,
};

// SPI Buffers
static uint8_t rx_buffer[BUF_SIZE];
static uint8_t tx_buffer[BUF_SIZE];

// Polling signal for SPI transaction completion
static struct k_poll_signal spi_done_sig;
// callback for SPI recieved data
static spi_rx_callback_t user_spi_callback = NULL;

/*
Setup SPI Device and DRDY Pin
*/
void spi_init(void)
{
    if (!device_is_ready(spi_dev))
    {
        printk("SPI device not ready!\n");
        return;
    }

    // NEW: Initialize DRDY pin as an output, starting LOW (Inactive)
    if (!gpio_is_ready_dt(&drdy_pin))
    {
        printk("DRDY pin not ready!\n");
        return;
    }
    gpio_pin_configure_dt(&drdy_pin, GPIO_OUTPUT_INACTIVE);

    printk("SPI API Initialized.\n");
}

void spi_set_callback(spi_rx_callback_t cb)
{
    user_spi_callback = cb;
}

/*
sets tx buffer and raises DRDY pin to trigger the SPI transaction with the master.
Requires that the spi_listener_thread is already running and waiting with the DMA armed, default when used with spi_init().
*/
void spi_send(const char *message)
{
    // 1. Clear out the old data in the transmit buffer
    memset(tx_buffer, 0, BUF_SIZE);

    // 2. Copy the new message in safely
    strncpy(tx_buffer, message, BUF_SIZE - 1);

    // 3. Set the DRDY pin HIGH to interrupt the Teensy Master
    // Since the SPI thread is already waiting with the DMA armed,
    // the Teensy will immediately clock this tx_buffer out.
    printk("[SPI] Sent: %s\n", tx_buffer);
    gpio_pin_set_dt(&drdy_pin, 1);
}

/*
Background thread that listens for SPI using DMA
*/
static void spi_listener_thread(void *arg1, void *arg2, void *arg3)
{
    k_poll_signal_init(&spi_done_sig);

    // Setup Receive Buffers
    struct spi_buf rx_buf = {.buf = rx_buffer, .len = sizeof(rx_buffer)};
    struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

    // Setup Transmit Buffers
    struct spi_buf tx_buf = {.buf = tx_buffer, .len = sizeof(tx_buffer)};
    struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

    struct k_poll_event events[1] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &spi_done_sig)};

    while (1)
    {
        // Arm BOTH the TX and RX buffers via DMA
        spi_transceive_signal(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs, &spi_done_sig);
        // Sleep the thread until the Master provides the clock cycles
        k_poll(events, 1, K_FOREVER);
        // at this point SPI transceive is complete

        // set DRDY pin LOW
        gpio_pin_set_dt(&drdy_pin, 0);

        // clear tx buffer for next message
        memset(tx_buffer, 0, BUF_SIZE);

        // process data from rx buffer
        if (user_spi_callback != NULL)
        {
            user_spi_callback(rx_buffer, BUF_SIZE);
        }

        // reset polling signals for the next loop
        k_poll_signal_reset(&spi_done_sig);
        events[0].state = K_POLL_STATE_NOT_READY;
    }
}

K_THREAD_DEFINE(spi_tid, 1024, spi_listener_thread, NULL, NULL, NULL, 7, 0, 0);