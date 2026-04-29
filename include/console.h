#ifndef CONSOLE_H
#define CONSOLE_H

/* * Define a "Function Pointer" type.
 * This says: a console_callback_t is any function that takes a 'const char *' and returns void.
 */
typedef void (*console_callback_t)(const char *cmd);

/* Public API Functions */
void console_init(void);
void console_set_callback(console_callback_t cb);

void spi_send(const char *message);

#endif /* CONSOLE_H */