#include <stdio.h>
#include <stdbool.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <string.h>

typedef struct {
    float pos;
    float vel;
    float torq;
} feedback;

esp_err_t init_can_driver(gpio_num_t tx_gpio, gpio_num_t rx_gpio);
esp_err_t uninstall_can_driver();
esp_err_t start_can_driver();
esp_err_t stop_can_driver();
esp_err_t send_msg(uint32_t id, uint8_t *data, uint8_t length, bool extended, uint32_t timeout_ms);
twai_message_t receive_msg(uint32_t timeout_ms);
void set_pos(float pos, float vel, float torq, float vel_scale, float torq_scale, uint32_t timeout_ms);
void set_pos_no_scale(float pos, float vel, float torq, uint32_t timeout_ms);
void set_vel(float vel, float torq, uint32_t timeout_ms);
void set_torque(float torq, uint32_t timeout_ms);
feedback get_encoder_estimates(uint32_t timeout_ms);
float get_torque_estimate(uint32_t timeout_ms);