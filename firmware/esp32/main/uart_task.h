#include <driver/uart.h>

#define buf_size 1024
#define cmd_buf_size 64

void init_uart(uart_port_t port);
char* uart_read(uart_port_t port, uint32_t timeout_ms);