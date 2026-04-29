#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "console.h"
#include "spi.h"

void handle_console_input(const char *input)
{
    printk("[MAIN] Console received: %s\n", input);
    spi_send(input);
}

void handle_spi_output(const uint8_t *data, uint32_t len)
{
    bool is_dummy = true;

    // Cast the pointer so the CPU reads 4 bytes per operation
    const uint32_t *word_data = (const uint32_t *)data;

    // We loop based on words (len / 4). For a 32-byte buffer, this loops 8 times.
    uint32_t word_count = len / 4;

    for (uint32_t i = 0; i < word_count; i++)
    {
        if (word_data[i] != 0xFFFFFFFF)
        {
            is_dummy = false;
            break;
        }
    }

    if (is_dummy)
    {
        return;
    }
    printk("[SPI] Received: %s\n", data);
}

int main(void)
{
    console_init();
    spi_init();
    console_set_callback(handle_console_input);
    spi_set_callback(handle_spi_output);
    return 0;
}