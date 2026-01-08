/*
TWAI == CAN. esp just refers CAN as TWAI.
refer this official doc: https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/peripherals/twai.html

Also, you must open 'this' projct in vs code, not via Exo_GUI. Otherwie vs code does not recognize the esp libraries.
*/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "can_driver.h"
#include "uart_task.h"

//TODO: configure gpio
#define tx_gpio GPIO_NUM_0
#define rx_gpio GPIO_NUM_1
#define uart_port UART_NUM_0

void app_main(void)
{
    //init can driver 
    if (init_can_driver(tx_gpio, rx_gpio) != ESP_OK) {
        printf("Driver istalled\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //start driver
    if (start_can_driver() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }

    //TODO: send command in a loop
    const uint32_t timeout_ms = 10;
    float target_pos = 0.0, target_vel = 0.0, target_torq = 0.0; 

    while (1) {
        //
        feedback fb = get_encoder_estimates(1);
        fb.torq = get_torque_estimate(1);

        //display the data to gui
        char buf[64];
        int n = sniprintf(buf, sizeof(buf), "%f,%f,%f\n", fb.pos, fb.vel, fb.torq);
        uart_write_bytes(uart_port, buf, n);


        //read inputs from gui and send them to driver
        char *cmd_buf = uart_read(uart_port, 10);
        if (strlen(cmd_buf) > 0) {
            switch (cmd_buf[0])
            {
            case 'T': 
                target_torq = atof(cmd_buf + 1);
                set_torque(target_torq, timeout_ms);
                break;
            case 'V':
                target_vel = atof(cmd_buf + 1);
                set_vel(target_vel, target_torq, timeout_ms);
                break;
            case 'P':
                target_pos = atof(cmd_buf + 1);
                set_pos_no_scale(target_pos, target_vel, target_torq, timeout_ms);
                break;
            default:
                break;
            }
        }
    }
}
