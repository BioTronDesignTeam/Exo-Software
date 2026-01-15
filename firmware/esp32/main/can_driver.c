#include "can_driver.h"

esp_err_t init_can_driver(gpio_num_t tx_gpio, gpio_num_t rx_gpio) {
    //init configs
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //install TWAI driver
    return twai_driver_install(&g_config, &t_config, &f_config);
}

esp_err_t uninstall_can_driver() {
    return twai_driver_uninstall();
}

esp_err_t start_can_driver() {
    return twai_start();
}

esp_err_t stop_can_driver() {
    return twai_stop();
}

/**
 * @brief Send a CAN message (command) via TWAI
 * 
 * @param id       CAN ID (standard 11-bit or extended 29-bit)
 * @param data     Pointer to data bytes to send
 * @param length   Number of bytes in data (0-8) (size of data)
 * @param extended  0 = standard frame, 1 = extended frame
 * @param timeout_ms Timeout in milliseconds for sending
 */
 esp_err_t send_msg(uint32_t id, uint8_t *data, uint8_t length, bool extended, uint32_t timeout_ms) {
    //configure message to transmit
    twai_message_t msg;
    msg.identifier = id;
    msg.extd = extended;
    msg.data_length_code = length;
    memcpy(msg.data, &data, length);

    //queue message for transmission
    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(timeout_ms));
 
    return res;
}

twai_message_t receive_msg(uint32_t timeout_ms) {
    //wait for message to be received
    twai_message_t msg;
    esp_err_t res = twai_receive(&msg, pdMS_TO_TICKS(timeout_ms));
        
    if (res != ESP_OK) {
        printf("Failed to receive message\n");
        msg.identifier = -1;
    }

    return msg;
}

void set_pos(float pos, float vel, float torq, float vel_scale, float torq_scale, uint32_t timeout_ms) {
    uint8_t data[8]; //always send 8 bytes data

    //allocate pos float into first 4 bytes of data (float is 4 bytes)
    memcpy(&data[0], &pos, 4);

    //desired feedforward velocity
    int16_t vel_ff = (int16_t) (vel * vel_scale);
    //split 16 bits into 8 bits
    data[4] = vel_ff & 0xFF; //store the lower 8 bits
    data[5] = (vel_ff >> 8) & 0xFF; //shift 8 bits to the right and store the upper 8 bits

    //desired feedforward torque
    int16_t torq_ff = (int16_t) (torq * torq_scale);
    data[6] = torq_ff & 0xFF;
    data[7] = (torq_ff >> 8) & 0xFF;

    send_msg(0x0c, data, 8, false, timeout_ms);
}

void set_pos_no_scale(float pos, float vel, float torq, uint32_t timeout_ms) {
    set_pos(pos, vel, torq, 1.0, 1.0, timeout_ms);
}

void set_vel(float vel, float torq, uint32_t timeout_ms) {
    uint8_t data[8]; 
    memcpy(&data[0], &vel, 4); //float is 4 bytes
    memcpy(&data[4], &torq, 4);
    send_msg(0x0d, data, 8, false, timeout_ms);
}

void set_torque(float torq, uint32_t timeout_ms) {
    uint8_t data[8];
    memcpy(&data[0], &torq, 4);
    send_msg(0x0e, data, 8, false, timeout_ms);
}

feedback get_encoder_estimates(uint32_t timeout_ms) {
    feedback val = {0};
    
    //send reqeust for encoder values (pos and vel) 
    if (send_msg(0x09, NULL, 0, false, timeout_ms) != ESP_OK) {
        return val;
    }
    
    //receive message
    twai_message_t rx = receive_msg(timeout_ms);
    if (rx.identifier == -1) {
        return val;
    }

    memcpy(&val.pos, &rx.data[0], 4);
    memcpy(&val.vel, &rx.data[4], 4);
    return val;
}

float get_torque_estimate(uint32_t timeout_ms) {
    if (send_msg(0x1c, NULL, 0, false, timeout_ms) != ESP_OK) {
        return 0.0;
    }

    twai_message_t rx = receive_msg(timeout_ms);
    if (rx.identifier == -1) return 0.0;
    
    float val = 0.0;
    //first 4 bytes: target torque, last 4 bytes: torque estimate
    memcpy(&val, &rx.data[4], 4);
    return val;
}