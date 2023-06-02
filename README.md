# ZDTMotor库使用指南

## 简介

ZDTMotor库是一个专为ESP-IDF平台设计的组件，用于操作ZDT系列（来自张大头品牌）的FOC闭环步进电机。当前版本主要支持UART串口通讯进行控制，而未来版本将增加对CAN协议通讯控制的支持。此外，我们也计划在未来扩展其支持的平台，包括arduino平台、platformIO平台。

ZDTMotor库的目标是提供一种简便的方式来读取、控制步进电机的各种属性、动作，包括读取步进电机的编码器和误差值，以及控制电机的方向、速度和加速度等。我们在库中封装了大量的十六进制信息交换逻辑，以简化控制流程，让使用者可以轻松快速地上手使用ZDT系列FOC闭环步进电机。

## 特性
ZDTMotor库的设计理念是将POP（面向过程编程）转为OOP（面向对象编程），我们尽力将复杂的底层控制逻辑进行封装，让你可以更加专注于你的应用开发。无论你是要简单地改变电机的方向，还是要进行更复杂的速度和加速度控制，ZDTMotor库都能为你提供易于使用的接口。~~甚至对于十六进制信息交换的处理，我们也已经将其完全封装，让你无需关注这些繁琐的细节。~~(暂未实现)

ZDTMotor库的开发是为了解决在使用ZDT系列闭环步进电机时可能遇到的各种问题。我们深知开发者在开发过程中可能面临的挑战，因此我们设计了这个库，希望能为你的开发过程提供便利。

## 如何使用

#### 直接基于本项目开发：
本项目是一个已测试过可用的库文件开发环境，懒人友好，可以直接使用本文档进行开发新项目。另外，如果在安装库文件的过程中遇到错误，你同样可以通过检查并对比本项目的配置来定位你配置的问题。
主函数中预置了测试代码，可以直接测试是否可用。


#### 转移库文件：
要在你的项目中使用ZDTMotor库，你需要执行以下步骤：
将本项目中components文件夹下的ZDTMotor库（也就是zdt_motor文件夹）复制到你的项目的components文件夹中。例如，如果你的项目源文件夹路径是/Users/your_name/esp/projects/your_project/components，那么你可以将zdt_motor文件夹复制到该路径下。


在你的**项目的**CMakeLists.txt文件中，（注意：不是main文件夹下的CMakeList.txt!!）确保包含以下两句语句：

```CMakeList.txt
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS "zdt_motor")
```

在你的源代码文件中(可能是main.cpp)，使用 ` #include "zdt_motor.h" ` 来引用ZDTMotor库。例如：

```cpp
#include "zdt_motor.h"
ZDTMotor myMotor(1, GPIO_NUM_4, GPIO_NUM_5);  // （地址，用于发送的GPIO引脚号，用于接收的GPIO引脚号）
```

## 常见问题与解决方案

#### **F1:** 编译器无法找到"driver/uart.h"？

**Q1:** 这个问题可能是由于在组件的 CMakeLists.txt 文件中没有包含 ESP-IDF 的路径，导致编译器无法找到 ESP-IDF 的头文件。

你可以尝试以下步骤来解决这个问题：
在你的项目目录（即包含 "main" 文件夹的那个目录）中找到 "CMakeLists.txt" 文件，然后添加下面这行代码：
```
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
```
这行代码将包含 ESP-IDF 的路径，从而允许编译器找到 ESP-IDF 中的头文件。
在你的 "zdt_motor" 组件的 "CMakeLists.txt" 文件中，确认你已经添加了以下这些行：
```
idf_component_register(SRCS "zdt_motor.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver)
```
其中，"SRCS" 参数列出了组件的源文件，"INCLUDE_DIRS" 参数指定了头文件的位置（在这种情况下，头文件在同一目录下，所以使用了 "."），"REQUIRES" 参数列出了组件依赖的其他组件（在这种情况下，是 ESP-IDF 的 "driver" 组件）。
如果你已经按照以上步骤操作但仍然遇到问题，请确保你的 IDF_PATH 环境变量已经正确设置为 ESP-IDF 的路径。你可以在终端中输入 "echo $IDF_PATH" 命令来检查 IDF_PATH 环境变量是否设置正确。

\n
#### **F2:** 函数的返回值不符合预期？发送的数据不是我想发送的？
**Q1:** 函数返回值与传入函数的参数都是**16进制的数据**，并且**一个参数仅能接受一个字节的数据**（也就是两位数的16进制数，如0xFF）。如果你传入了10进制的数据，或者将16进制返回值当作10进制对待，数值当然是错误的。
也就是说，你需要将数据转换为10进制即可。
在未来，库中会另外实现一个函数，用于“16进制 - 10进制”的互转。
当然，如果大家一致认为在原函数中就实现十进制比较合理，后期可以考虑改写各个函数。

## 联系与支持

欢迎你使用ZDTMotor库，如果你在使用ZDTMotor库过程中遇到了问题，或者有任何建议或反馈，欢迎提交Issue或者Pull Request，我们都非常愿意听取。让我们一起让ZDT系列的FOC闭环步进电机使用起来更加简单方便！

## 许可

ZDTMotor库在 Apache 2.0 许可下发布。详见LICENSE文件。

## ZDTMotor && 未来
虽然ZDTMotor库现在已经具备了一些基本功能，但我们的工作还远远没有结束。我们正在计划添加更多的特性和优化，包括对CAN协议的支持，以及扩展到其他的平台，如arduino。我们期待在未来的版本中为你提供更多的便利和帮助。

感谢你使用ZDTMotor库，祝您编程愉快！