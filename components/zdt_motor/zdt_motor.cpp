/*
 * Copyright 2023 Meieryang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Github URL: git@github.com:Meieryang/zdt-motor-lib.git
 *
 * Software version: 0.1beta
 */

#include "zdt_motor.h"

ZDTMotor::ZDTMotor (uint8_t adress, gpio_num_t gpio_num_tx, gpio_num_t gpio_num_rx,
            uint16_t buf_size,
            int baud_rate_config, uint8_t uart_num_which,
            gpio_num_t gpio_num_uart_rts, 
            gpio_num_t gpio_num_uart_cts,
            uint8_t data_check_byte) : 
           adress(adress), gpio_num_tx(gpio_num_tx), gpio_num_rx(gpio_num_rx),
           buf_size(buf_size),
           baud_rate_config(baud_rate_config), uart_num_which(uart_num_which),
           gpio_num_uart_rts(gpio_num_uart_rts), gpio_num_uart_cts(gpio_num_uart_cts),
           data_check_byte(data_check_byte)
    {
        init();
    }


void ZDTMotor::init()
{
const uart_config_t uart_config =
{
    .baud_rate = baud_rate_config,  // 波特率
    .data_bits = UART_DATA_8_BITS,  // 数据位长度
    .parity = UART_PARITY_DISABLE,  // 奇偶校验位
    .stop_bits = UART_STOP_BITS_1,  // 停止位长度
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 硬件流控制模式
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB  // 时钟源 /APB: 默认高性能时钟
};

//configure UART parameters
ESP_ERROR_CHECK(uart_param_config(uart_num_which, &uart_config));  // ESP_ERROR_CHECK() is a macro,not a function
uart_set_pin(uart_num_which, gpio_num_tx, gpio_num_rx, gpio_num_uart_rts, gpio_num_uart_cts);
uart_driver_install(uart_num_which, buf_size * 2, 0, 0, NULL, 0);

}


template<typename... Args>
bool ZDTMotor::send_data(cmd_t cmd, Args... args)
{
    std::vector<uint8_t> hex_data = {adress, cmd};
    (hex_data.push_back(args), ...);
    hex_data.push_back(data_check_byte);
    uart_write_bytes(uart_num_which, hex_data.data(), hex_data.size());
    return true;
}


int32_t ZDTMotor::receive_data(int8_t expected_length)
{
    uint8_t rx_buffer[expected_length];
    int8_t length = (int8_t)uart_read_bytes(uart_num_which, rx_buffer, sizeof(rx_buffer), 100 / portTICK_PERIOD_MS);

    if (length == expected_length)
    {
        if (rx_buffer[0] != adress)
        {
            ESP_LOGE("ZDTMotor", "地址错误");
            return RT_WARNING;
        }
        if (rx_buffer[expected_length - 1] != data_check_byte)
        {
            ESP_LOGE("ZDTMotor", "校验码错误");
            return RT_WARNING;
        }
        // 在这段代码中，receivedData是在循环中从rx_buffer读取数据，并将每个字节拼接到一起得到一个最终的整数值。
        // 对于不同的expected_length值，接收到的数据长度会有所不同。
        // expected_length = 3：在这种情况下，receivedData将等于rx_buffer[1]，因为只有一次循环。

        // expected_length = 4：这时，receivedData将等于((uint16_t)rx_buffer[1] << 8) | rx_buffer[2]。
        // 这是因为在两次循环中，首先将rx_buffer[1]左移8位，然后将rx_buffer[2]的值与其进行或运算，形成一个16位的值。

        // expected_length = 6：在这种情况下，receivedData将等于((uint32_t)rx_buffer[1] << 24) | ((uint32_t)rx_bu
        // ffer[2] << 16) | ((uint32_t)rx_buffer[3] << 8) | ((uint32_t)rx_buffer[4])。这是因为在四次循环中，我们分别将rx_buffer[1]，rx_buffer[2]，rx_buffer[3]和rx_buffer[4]的值进行左移并进行或运算，形成一个32位的值。
        // 以上所述，这段代码的作用是将缓冲区中的数据按照大端序合成为一个整型数。
        int32_t receivedData = 0;
        for(int i = 1; i < expected_length - 1; i++) {
            receivedData = (receivedData << 8) | rx_buffer[i];
        }
        return receivedData;
    }

    // 在这里处理数据长度不符合预期的情况
    ESP_LOGE("ZDTMotor", "数据长度错误");
    return RT_WARNING;
}



uint8_t ZDTMotor::cancel_motor_protect() {

    send_data(CMD_CANCEL_MOTOR_PROTECT_P1, CMD_CANCEL_MOTOR_PROTECT_P2);
    return receive_data(_DEFAULT_BYTES);
}


uint16_t ZDTMotor::get_encoder_value() {
    
    send_data(CMD_GET_ENCODER_VALUE);
    return receive_data(_4_BYTES);  // receive the data that contains 4 bytes
}


uint32_t ZDTMotor::get_pulse_value() {

    send_data(CMD_GET_PULSE_VALUE);
    return receive_data(_6_BYTES);  // receive the data that contains 6 bytes
}


uint32_t ZDTMotor::get_motor_position() {

    send_data(CMD_GET_MOTOR_POSITION);
    return receive_data(_6_BYTES);
}


uint16_t ZDTMotor::get_motor_position_err() {
    
    send_data(CMD_GET_MOTOR_POSITION_ERR);
    return receive_data(_4_BYTES);
}


uint8_t ZDTMotor::get_en_status() {

    send_data(CMD_GET_EN_STATUS);
    return receive_data(_3_BYTES);
}


uint8_t ZDTMotor::get_protect_watermark() {

    send_data(CMD_GET_PROTECT_WATERMARK);
    return receive_data(_3_BYTES);
}


uint8_t ZDTMotor::get_o_status() {

    send_data(CMD_GET_O_STATUS);
    return receive_data(_3_BYTES);
}

uint8_t ZDTMotor::edit_m_step(uint8_t m_step) {
    send_data(CMD_EDIT_M_STEP, m_step);
    return receive_data(_DEFAULT_BYTES);
}

uint8_t ZDTMotor::edit_uart_addr(uint8_t addr) {
    send_data(CMD_EDIT_UART_ADDR, addr);
    return receive_data(_DEFAULT_BYTES);
}

uint8_t ZDTMotor::ctrl_en(uint8_t enabled) {
    send_data(CMD_CTRL_EN, enabled);
    return receive_data(_DEFAULT_BYTES);
}

uint8_t ZDTMotor::ctrl_speed_mode(uint8_t direction, uint8_t speed, uint8_t acceleration) {

    send_data(CMD_CTRL_SPEED_MODE, direction, speed, acceleration);
    return receive_data(_DEFAULT_BYTES);
}

uint8_t ZDTMotor::clear_speed_mode() {

    send_data(CMD_CLEAR_SPEED_MODE_P1, CMD_CLEAR_SPEED_MODE_P2);
    return receive_data(_DEFAULT_BYTES);
}

uint8_t ZDTMotor::save_speed_mode() {

    send_data(CMD_SAVE_SPEED_MODE_P1, CMD_SAVE_SPEED_MODE_P2);
    return receive_data(_DEFAULT_BYTES);
}

uint8_t ZDTMotor::ctrl_pos_mode(uint8_t direction, uint8_t speed, uint8_t acceleration, uint8_t pulse1, uint8_t pulse2, uint8_t pulse3) {

    send_data(CMD_CTRL_POS_MODE, direction, speed, acceleration, pulse1, pulse2, pulse3);
    return receive_data(_DEFAULT_BYTES);
}



