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

#include <stdio.h>
#include "zdt_motor.h"

// the output sample:
// c053
// 0
// fffffff8
// 1
// 1
void get_info_list(ZDTMotor& motor_obj) {

    printf("encoder value: %x",motor_obj.get_encoder_value());   printf("\n");
    printf("pulse value: %lx",motor_obj.get_pulse_value());   printf("\n");
    printf("motor position: %lx",motor_obj.get_motor_position());   printf("\n");
    printf("motor position err: %x",motor_obj.get_motor_position_err());   printf("\n");
    printf("en status: %x",motor_obj.get_en_status());   printf("\n");
    printf("\n");
}


/*
*使用 extern "C" 是连接 C++ 和 C 语言的一种常见方法，特别是在你的项目中混合使用这两种语言的时候。
*当你在 C++ 文件中声明 extern "C" 时，这告诉 C++ 编译器按照 C 语言的方式来处理被声明的代码。
*/
extern "C"
void app_main(void)
{
    vTaskDelay(120);
    ZDTMotor arm(1, GPIO_NUM_4, GPIO_NUM_5);
    get_info_list(arm);
    arm.ctrl_speed_mode(0x10,0x22,0x00);

    for (;;) {

    vTaskDelay(50);
    get_info_list(arm);
    }
}
