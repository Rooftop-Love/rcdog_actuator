# Dog Actuator Firmware

这是一个用于 RC 狗执行板的 STM32F103C8Tx 固件。它通过简洁的 UART
命令/响应协议，控制真空泵、舵机和 8 路音频触发模块。

## 硬件

- MCU：STM32F103C8Tx，Cortex-M3
- 存储：64 KB Flash，20 KB RAM
- 板载 LED：PC13
- 舵机 PWM：PA0，TIM2_CH1
- 泵 PWM：PA6，TIM3_CH1
- 继电器：在 CubeMX 中配置为 `Relay_Pin`
- 音频输出：PB9、PB8、PB7、PB6、PB5、PB4、PB3、PA15

音频引脚为低电平有效。播放时会把对应引脚拉低 `AUDIO_PULSE_MS`
毫秒，然后恢复为高电平。

## 构建

先安装 CMake、Ninja 和 `arm-none-eabi-gcc`，然后执行：

```sh
cmake --preset Debug
cmake --build build/Debug
```

Debug 产物为：

```text
build/Debug/dog_actuator.elf
```

如果要生成体积更小的量产版本：

```sh
cmake --preset Release
cmake --build build/Release
```

默认 preset 使用 `cmake/gcc-arm-none-eabi.cmake`。仓库里还有
`cmake/starm-clang.cmake`，但它没有接入 presets。

## 固件流程

`Core/Src/main.c` 会先初始化 CubeMX 外设，调用 `Protocol_Init()`，
然后进入如下非阻塞循环：

```c
while (1)
{
  Protocol_Process();
  Action_Tick();
}
```

主要模块：

- `Core/Src/protocol.c`：UART 帧接收状态机、CRC 校验、命令去重、分发和响应发送。
- `Core/Src/action.c`：执行器状态机和物理输出控制。
- `Core/Inc/protocol.h`：帧格式、命令码和状态码。
- `Core/Inc/action.h`：动作层接口。
- `Drivers/`：CubeMX 生成的 STM32 HAL/LL 驱动。

## UART 协议

USART2 是命令口。USART1 也支持同样的协议，可用于调试或台架测试。

请求帧，12 字节：

```text
0xA5 0x00 | cmd_id[2] | cmd[2] | param1[2] | param2[2] | crc16[2]
```

响应帧，8 字节：

```text
0xA5 0x00 | cmd_id[2] | status[2] | crc16[2]
```

所有多字节字段均为小端序。CRC 使用 CRC-16/MODBUS，参数为
`0xA001`，初值 `0xFFFF`，输入/输出均反射。请求帧 CRC 覆盖第 0 到
9 字节；响应帧 CRC 覆盖第 0 到 5 字节。

重复的 `cmd_id` 会被去重：固件只返回当前状态，不会再次执行命令。

## 命令

| 命令 | 值 | 参数 | 行为 |
| --- | ---: | --- | --- |
| `CMD_SUCK` | `0x0001` | 无 | 启动吸附流程。 |
| `CMD_RELEASE` | `0x0002` | 无 | 启动释放流程。 |
| `CMD_CANCEL` | `0x0003` | 无 | 立即中止当前任务和音频脉冲。 |
| `CMD_MOVE` | `0x0100` | `param1=x`，`param2=y` | 预留的移动命令；当前会把舵机复位到 0 度。 |
| `CMD_QUERY` | `0x0200` | 无 | 返回当前动作状态。 |
| `CMD_PLAY_AUDIO` | `0x0300` | `param1=1..8` | 拉低指定音频输出。 |

## 状态码

| 状态 | 值 | 含义 |
| --- | ---: | --- |
| `STATUS_IDLE` | `0x0000` | 当前无任务。 |
| `STATUS_EXECUTING` | `0x0001` | 任务正在执行。 |
| `STATUS_OK` | `0x0002` | 命令或任务执行成功。 |
| `STATUS_FAIL_NO_BLOCK` | `0x0003` | 未知命令或不支持的操作。 |
| `STATUS_FAIL_TIMEOUT` | `0x0004` | 协议预留的超时状态。 |
| `STATUS_FAIL_MOTOR` | `0x0005` | 协议预留的电机故障状态。 |

## CubeMX 注意事项

CubeMX 生成的文件中有 `USER CODE BEGIN` / `USER CODE END` 区域。
自定义代码请放在这些区域内，否则重新生成时可能被覆盖。

`Core/Src/protocol.c` 和 `Core/Src/action.c` 是从根目录
`CMakeLists.txt` 中加入的用户代码文件，可以直接编辑。

## 验证

当前没有测试框架或 CI。通常的验证流程是：

1. 使用 Debug preset 配置并构建。
2. 将 `dog_actuator.elf` 烧录到目标板。
3. 在 USART2 上发送合法 UART 帧，检查响应、泵 PWM、舵机 PWM、
   继电器输出和音频脉冲是否符合预期。
