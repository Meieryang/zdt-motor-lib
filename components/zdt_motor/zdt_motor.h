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

#pragma once

#include <vector>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// return status
#define RT_WARNING 0
#define RT_OK 1
#define RT_ERROR -1


typedef enum {

    _DEFAULT_BYTES = 3,  // default return, i.e. 0x02(success) & 0xEE (fail)
    _3_BYTES = 3,
    _4_BYTES = 4,
    _6_BYTES = 6,
    _8_BYTES = 8,
} byte_num_t;

// commands, DO NOT modify it !!
typedef enum {

    // __GET_CMDS
    CMD_GET_ENCODER_VALUE = 0x30,
    CMD_GET_PULSE_VALUE = 0x33,
    CMD_GET_MOTOR_POSITION = 0x36,
    CMD_GET_MOTOR_POSITION_ERR = 0x39,
    CMD_GET_EN_STATUS = 0x3A,
    CMD_GET_PROTECT_WATERMARK = 0x3E,
    CMD_GET_O_STATUS = 0x3F,
    // __EDIT_CMDS
    CMD_EDIT_M_STEP = 0x84,
    CMD_EDIT_UART_ADDR = 0xAE,
    // __CTRL_CMDS
    CMD_CTRL_EN = 0xF3,
    CMD_CTRL_SPEED_MODE = 0xF6,
    CMD_CTRL_POS_MODE = 0xFD,
    // 
    CMD_CANCEL_MOTOR_PROTECT_P1 = 0x0E,
    CMD_CANCEL_MOTOR_PROTECT_P2 = 0x52,
    CMD_CLEAR_SPEED_MODE_P1 = 0xFF,
    CMD_CLEAR_SPEED_MODE_P2 = 0xCA,
    CMD_SAVE_SPEED_MODE_P1 = 0xFF,
    CMD_SAVE_SPEED_MODE_P2 = 0xC8
} cmd_t;

class ZDTMotor {
private:
    uint8_t adress;
    gpio_num_t gpio_num_tx, gpio_num_rx;
    uint16_t buf_size;
    int baud_rate_config;
    uint8_t uart_num_which;
    gpio_num_t gpio_num_uart_rts, gpio_num_uart_cts;
    uint8_t data_check_byte;  // 默认值 = 0x6B

public:

    /**
     * @brief 初始化ZDT电机库
     *
     * 该构造函数用于初始化ZDT电机库，你需要在这里配置所需的GPIO引脚、缓冲区大小、通讯波特率、UART编号以及RTS和CTS的GPIO编号。
     *
     * @param gpio_num_tx 用于发送数据的GPIO引脚编号
     * @param gpio_num_rx 用于接收数据的GPIO引脚编号
     * @param buf_size 缓冲区大小，默认值为1024
     * @param baud_rate_config 波特率配置，默认值为115200
     * @param uart_num_which UART编号，此配置用于选择你想要使用的通用异步收发传输器（UART），默认值为UART_NUM_1，
     * 可选的选项有 `UART_NUM_0` `UART_NUM_1` `UART_NUM_2`。
     * ESP32包含三个UART（通用异步收发传输器）模块，分别是UART0、UART1和UART2。这些UART模块可以映射到
     * 任何GPIO（通用输入输出）引脚上。
     * 每个UART模块都可以独立配置并进行串行通信，包括发送和接收数据。这些UART模块可以使用ESP-IDF中的函数进行
     * 配置，例如设置波特率、数据位、停止位、奇偶校验位等。
     * 注意，UART_NUM_0为开发版与电脑通讯的UART，请尽量不要占用，以免影响通讯以及烧录。
     * @param gpio_num_uart_rts UART的RTS使用哪个GPIO引脚编号，默认值为GPIO_NUM_NC，表示不使用硬件流控制的请求发送。
     * 通常，硬件流控制用于防止数据溢出，通过控制数据的发送速度来保证接收方的缓冲区不会被溢出。但在某些情况下，比如
     * 数据量较小，或者发送和接收速度较慢，就可能不需要使用硬件流控制。这时就可以将RTS和CTS引脚设置为GPIO_NUM_NC，
     * 即不使用这两个引脚，从而禁用硬件流控制。
     * @param gpio_num_uart_cts UART的CTS使用哪个GPIO引脚编号，默认值为GPIO_NUM_NC，表示不使用硬件流控制的清除发送。
     */
    ZDTMotor (uint8_t adress, gpio_num_t gpio_num_tx, gpio_num_t gpio_num_rx,
            uint16_t buf_size = 1024,
            int baud_rate_config = 115200, uint8_t uart_num_which = UART_NUM_1,
            gpio_num_t gpio_num_uart_rts = GPIO_NUM_NC, 
            gpio_num_t gpio_num_uart_cts = GPIO_NUM_NC,
            uint8_t data_check_byte=0x6B);

private:

    void init();

    /**
     * @brief （私有函数）发送数据
     * 
     * 这是一个私有的内部函数，用途是复用一些类内公开函数内部的代码行，因此在函数外部无法访问。
     * 
     * @param cmd 接收一个自定义枚举类型cmd_t，声明在开头。用于根据传入的命令向制定地址发送十六进制数据。
     * @param Args 接收任意个大小为1个字节的十六进制参数，按顺序输入，本函数会自动按参数顺序将参数发送出去。
     * @return bool 如果程序正常运行到函数行尾，返回真。反之返回假。注意：这个返回值不会检测对方是否真的已经收到了数据，
     *              仅仅表示数据已经被发送完成。关于 接收&检测 数据的逻辑，依靠于之后的代码。
    */
    template<typename... Args>
    bool send_data(cmd_t cmd, Args... args);

    /**
     * @brief （私有函数）接收数据
     * 
     * 这是一个私有的内部函数，用于复用一些类内公开函数内部的代码行，因此在函数外部无法访问。
     * 如果接收到正确的数据，返回一个整型数据。如果接收到的数据不符合预期，返回零。
     * 
     * @param expected_length 期望的数据长度。
     * @return int32_t 从缓冲区接收到的数据。
    */
    int32_t receive_data(int8_t expected_length);

public:

    /**
     * @brief 解除堵转保护
     * 
     * 只有触发了堵转保护功能后，下发该命令才有效，可以解除堵转保护；
     * 
     * @return 0x02(success) or 0xEE(fail)
    */
    uint8_t cancel_motor_protect();

    /**
     * @brief 读取编码器值
     * 
     * 读取位置传感器的数值
     * 
     * @return 编码器的值，范围为0-65535。返回2个字节，例如 0xee ff
    */
    uint16_t get_encoder_value();

    /**
     * @brief 读取输入脉冲数
     * 
     * 读取位置传感器的数值读取输入脉冲的数值，即对应小屏幕上的第3行的数据。
     * 
     * @return 数据类型为int32_t，范围为-2147483647-2147483647。返回4个字节，例如 0xcc dd ee ff
    */
    uint32_t get_pulse_value();

    /**
     * @brief 读取电机实时位置
     * 
     * 读取电机的实时位置，即对应小屏幕上的第1行的数据。电机转过的角度 =（电机位置 * 360）/ 65536
     * 
     * @return 数据类型为int32_t，范围为-2147483647-2147483647。返回4个字节，例如 0xcc dd ee ff
    */
    uint32_t get_motor_position();

    /**
     * @brief 读取电机位置误差
     * 
     * 读取电机设定位置与实际位置的差值，即对应小屏幕上的第2行的数据。
     * 
     * @return 数据类型为int16_t，范围为-32767-32767。返回2个字节，例如 0xee ff
    */
    uint16_t get_motor_position_err();

    /**
     * @brief 读取使能状态
     * 
     * 读取闭环电机的使能状态
     * 
     * @return 返回00 表示闭环电机处于不使能状态；
     * 返回 01 表示闭环电机处于使能状态；
    */
    uint8_t get_en_status();

    /**
     * @brief 读取堵转标志
     * 
     * 读取闭环电机的堵转标志
     * 

     * @return 返回 00 表示闭环电机当前没发生堵转；
     * 返回 01 表示闭环电机当前处于堵转状态；
    */
    uint8_t get_protect_watermark();

    /**
     * @brief 读取单圈上电自动回零状态标志
     * 
     * 读取单圈上电自动回零状态标志，返回数据类型为uint8_t，数值为00或01。
     * 
     * @return 返回 00 表示上电时闭环电机触发单圈上电自动回零状态正常；
     * 返回 01 表示上电时闭环电机触发单圈上电自动回零状态不正常，如回零过程堵转、回零方向设置错误等；
    */
    uint8_t get_o_status();

    /**
     * @brief 修改当前细分步数
     * 
     * 修改当前细分步数，该命令可以将闭环电机当前的细分步数修改为1-256任意细分步数，例如：
     * 发送01 84 00 6B修改闭环电机当前细分步数为256；
     * 发送01 84 03 6B修改闭环电机当前细分步数为3；
     * 发送01 84 FF 6B修改闭环电机当前细分步数为255；
     * 
     * @param m_step 输入需要更新的细分步数值
     * @return 0x02(success) or 0xEE(fail)
    */
    uint8_t edit_m_step(uint8_t m_step);

    /**
     * @brief 修改当前串口通讯地址
     * 
     * 修改当前串口通讯地址为16，该命令可以将闭环电机当前的串口通讯地址修改为1-247任意数值，例如：
     * 发送01 84 04 6B修改闭环电机当前串口通讯地址为4；
     * 发送01 84 46 6B修改闭环电机当前串口通讯地址为70；
     * 
     * @param addr 输入需要更新的串口通讯地址
     * @return 0x02(success) or 0xEE(fail)
    */
    uint8_t edit_uart_addr(uint8_t addr);

    /**
     * @brief 控制闭环电机的使能状态
     * 
     * 控制闭环电机的使能状态，例如：
     * 00 控制闭环电机处于状态；
     * 01 控制闭环电机处于使能状态；
     * 
     * @param enabled 输入需要更新的使能状态值。00 不使能；01 使能；
     * @return 0x02(success) or 0xEE(fail)
    */
    uint8_t ctrl_en(uint8_t enabled);

    /**
     * 
     * @brief 控制闭环电机的正反转，即速度模式控制
     * 
     * 命令格式：地址 + 0xF6 + 方向和速度（共用2个字节） + 加速度 + 校验字节
     * 命令示例：发送01 F6 14 FF 00 6B
     * 其中，0x14 0xFF两个字节表示方向和速度，最高的半字节0x1表示方向，剩下的0x4FF表示速度档位，最大为4FF，即1279个速度档位；方向和速度后面的字节0x00
     * 表示加速度档位，即启动和停止时的曲线加减速档位，加速度值如果是0xFF，即255，则不启用曲线加减速功能，直接以指定的速度去运行。如果需要控制闭环电机停止，可以将速度档位设置为0发送命令，例如，发送01 F6 10 00 00 6B或01 F6 00 00 00 6B都可以使闭环电机停止运动。
     * 命令作用：控制闭环电机按照设定的方向、速度和加速度进行一直旋转，加速度是指设置启动和停止时曲线加减速档位，可以减少电机启动和停止的震动，例如：
     * 发送01 F6 12 FF 00 6B可以使闭环电机以逆时针方向，0x2FF速度档位，0x00加速度档位一直旋转；
     * 发送01 F6 02 FF FF 6B可以使闭环电机以顺时针方向，0x2FF速度档位，0xFF加速度档位一直旋转；
     * 发送01 F6 10 00 00 6B可以使闭环电机以0x00加速度档位缓慢停止；
     * 发送01 F6 10 00 FF 6B可以使闭环电机立即停止；
    */
    uint8_t ctrl_speed_mode(uint8_t direction, uint8_t speed, uint8_t acceleration);

    /**
     * 
     * 清除闭环电机正反转，即速度模式当前的参数，上电会自动运行
     * 
     * 命令格式：地址 + 0xFF + 清除 + 校验字节
     * 命令作用：清除闭环电机正反转，即速度模式当前运行的的方向/速度/加速度参数，存储后下次上电会按照这些参数自动运行，例如：
     * 发送01 FF CA 6B清除闭环电机上一次存储的参数；
    */
    uint8_t clear_speed_mode();

    /**
     * 
     * 存储闭环电机正反转，即速度模式当前的参数，上电会自动运行
     * 
     * 命令格式：地址 + 0xFF + 存储 + 校验字节
     * 命令作用：存储闭环电机正反转，即速度模式当前运行的的方向/速度/加速度参数，存储后下次上电会按照这些参数自动运行，例如：
     * 发送01 FF CA 6B存储闭环电机上一次存储的参数；
    */
    uint8_t save_speed_mode();

    /**
     * 
     * @brief 控制闭环电机相对运动的角度，即位置模式控制
     * 
     * 命令格式：
     * 地址 + 0xFD + 方向和速度（2字节） + 加速度 + 脉冲数（3字节） + 校验字节
     * 命令示例：01 FD 14 FF 00 00 0C 80 6B
     * 其中，0x14 0xFF两个字节表示方向和速度，最高的半字节0x1表示方向，剩下的0x4FF表示速度档位，最大为4FF，即1279个速度档位；
     * 方向和速度后面的字节0x00表示加速度档位，即启动和停止时的曲线加减速档位，加速度值如果是0xFF，即255，则不启用曲线加减速功能，
     * 直接以指定的速度去运行。如果需要控制闭环电机缓慢停止，可以将速度档位设置为0，如果需要控制闭环电机立即停止，可以将速度档位设置为0，
     * 加速度档位设置为255；加速度档位后面的3个字节是脉冲数，例如16细分下发送3200个脉冲就可以让1.8°的电机转动一圈。
     * 命令作用：控制闭环电机按照设定的方向、速度和加速度进行相对位置运动，例如：
     * 发送01 FD 02 FF 00 00 0C 80 6B可以使闭环电机以顺时针方向，0x2FF速度档位，0x00加速度档位运动0x000C80个脉冲，即3200个脉冲；
     * 发送01 FD 12 FF 45 00 19 00 6B可以使闭环电机以顺时针方向，0x2FF速度档位，0x45加速度档位运动0x001900个脉冲，即6400个脉冲；
     * 发送01 FD 10 00 FF 00 19 00 6B可以使闭环电机缓慢停止（速度档位为0）；
     * 发送01 FD 12 FF FF 00 00 00 6B可以使闭环电机立即停止（脉冲为0，加速度为255）；
     * 注意：发送该命令后，闭环电机收到指令，会先返回01 02 6B的命令进行应答；紧接着，闭环电机内部更新完全部的脉冲（位置），
     * 会返回01 9F 6B的命令，表示闭环电机已经全部更新完脉冲（位置），如果不想返回，可在PcmdRet选项设置为Disable。
    */
    uint8_t ctrl_pos_mode(uint8_t direction, uint8_t speed, uint8_t acceleration, uint8_t pulse1, uint8_t pulse2, uint8_t pulse3);

};
