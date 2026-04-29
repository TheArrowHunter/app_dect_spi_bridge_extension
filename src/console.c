#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include "console.h"

#define RX_BUF_SIZE 64
static uint8_t rx_double_buf[2][RX_BUF_SIZE];
static char cmd_buf[RX_BUF_SIZE + 1];

const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
K_SEM_DEFINE(uart_rx_sem, 0, 1);

// This variable holds the function main() will pass to us
static console_callback_t user_callback = NULL;

void console_set_callback(console_callback_t cb) {
    user_callback = cb;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {
    case UART_RX_RDY:
        memcpy(cmd_buf, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
        cmd_buf[evt->data.rx.len] = '\0'; 
        k_sem_give(&uart_rx_sem);
        break;
    case UART_RX_BUF_REQUEST:
        uart_rx_buf_rsp(dev, rx_double_buf[1], RX_BUF_SIZE);
        break;
    case UART_RX_DISABLED:
        uart_rx_enable(dev, rx_double_buf[0], RX_BUF_SIZE, 1000);
        break;
    default:
        break;
    }
}

void console_thread_entry(void *arg1, void *arg2, void *arg3) {
    uart_callback_set(uart_dev, uart_cb, NULL);
    uart_rx_enable(uart_dev, rx_double_buf[0], RX_BUF_SIZE, 1000);
    
    while (1) {
        k_sem_take(&uart_rx_sem, K_FOREVER);
        
        // If main.c has registered a callback, trigger it and pass the string!
        if (user_callback != NULL) {
            user_callback(cmd_buf);
        }
    }
}

// Start the background thread automatically
K_THREAD_DEFINE(console_tid, 1024, console_thread_entry, NULL, NULL, NULL, 8, 0, 0);

// A simple init check you can call from main
void console_init(void) {
    if (!device_is_ready(uart_dev)) {
        printk("Console UART not ready!\n");
    }
}