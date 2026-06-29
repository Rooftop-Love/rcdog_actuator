# AGENTS.md

## Project overview

STM32F103C8Tx (Cortex-M3, 64K Flash, 20K RAM) firmware for an RC dog actuator board. Controls a vacuum pump, servo, and 8-channel audio module via a USART2 command/response protocol.

## Build

Requires `arm-none-eabi-gcc` on PATH (or ST's `cube-cmake` / `starm-clang` toolchain).

```
cmake --preset Debug
cmake --build build/Debug
```

Output: `build/Debug/dog_actuator.elf`

Use `Release` preset for `-Os` production builds. The CMake preset uses Ninja and the GCC toolchain by default (`cmake/gcc-arm-none-eabi.cmake`). An alternative Clang toolchain exists at `cmake/starm-clang.cmake` but is not wired into the presets.

## Code structure

- `Core/Src/main.c` — CubeMX-generated entry point. Calls `Protocol_Init()` then loops `Protocol_Process()` + `Action_Tick()`.
- `Core/Src/protocol.c` — USART2 frame protocol. ISR feeds bytes; main loop does CRC check, de-duplication, dispatch, and response transmit.
- `Core/Src/action.c` — Actuator state machine and physical outputs (pump, servo, audio).
- `Core/Inc/protocol.h` — Frame format, command codes, status codes.
- `Drivers/` — STM32 HAL/LL drivers (CubeMX-managed, do not edit).
- `cmake/stm32cubemx/CMakeLists.txt` — CubeMX-generated build glue. Sources listed here mirror the `.ioc`.
- `STM32F103XX_FLASH.ld` — Linker script (64K FLASH, 20K RAM).
- `dog_actuator.ioc` — STM32CubeMX project file. Regenerates HAL config, GPIO, timers, USART init.

## CubeMX code-generation contract

Files in `Core/Src/` and `Core/Inc/` contain `USER CODE BEGIN` / `USER CODE END` markers. CubeMX only touches code **outside** these markers. **Always place custom code inside the markers** or it will be overwritten on regeneration.

Files that CubeMX fully owns (`gpio.c`, `tim.c`, `usart.c`, `stm32f1xx_it.c`, `stm32f1xx_hal_msp.c`, `system_stm32f1xx.c`, `syscalls.c`, `sysmem.c`) — edit only inside markers.

User-authored files (`protocol.c`, `action.c`) are safe to edit freely; they are listed in the root `CMakeLists.txt` `target_sources`, not in the CubeMX subdirectory.

## Protocol (USART2)

- Request: 12 bytes — `0xA5 0x00 | cmd_id[2] | cmd[2] | param1[2] | param2[2] | crc16[2]`
- Response: 8 bytes — `0xA5 0x00 | cmd_id[2] | status[2] | crc16[2]`
- All multi-byte fields little-endian.
- CRC-16/MODBUS: poly 0xA001, init 0xFFFF, refin/refout.
- `cmd_id` de-duplication: repeated `cmd_id` returns current status without re-executing.

## Hardware pin mapping

| Function   | Pin     |
|------------|---------|
| LED_Board  | PC13    |
| Servo      | PA0 (TIM2_CH1) |
| Pump       | PA6 (TIM3_CH1) |
| Sound 1–8  | PB9, PB8, PB7, PB6, PB5, PB4, PB3, PA15 |

Sound pins are active-low (pulse LOW for `AUDIO_PULSE_MS` = 1000 ms, then return HIGH).

## Key conventions

- Language: C11 with extensions (`CMAKE_C_EXTENSIONS ON`). No C++ in this project.
- No test framework, no CI, no linting configured. Verification is manual flash-and-test.
- `.clangd` points at `build/Debug` for `compile_commands.json`. Run the Debug configure step before expecting IDE features to work.
- The `build/` directory is gitignored. Always configure before building.
