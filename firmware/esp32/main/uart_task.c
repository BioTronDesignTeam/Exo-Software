#include "uart_task.h"

void init_uart(uart_port_t port) {
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(port, &config);
    uart_driver_install(port, buf_size, 0, 0, NULL, 0);
}

//read input from gui
char* uart_read(uart_port_t port, uint32_t timeout_ms) {
    char cmd_buf[cmd_buf_size];
    uart_read_bytes(port, cmd_buf, cmd_buf_size, pdMS_TO_TICKS(timeout_ms));
    return cmd_buf;
}