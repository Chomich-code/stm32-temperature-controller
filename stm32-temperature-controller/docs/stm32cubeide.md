# STM32CubeIDE setup

This project source targets the **STM32 NUCLEO-F756ZG** board with STM32 HAL. The source file is intended to be copied into a working STM32CubeIDE project with the generated supporting files.

## Core settings

| Peripheral | Setting |
| --- | --- |
| SYS | Serial Wire debug, SysTick timebase |
| Clock | HSI 16 MHz; PLL disabled; AHB/APB1/APB2 dividers /1 |
| I2C1 | I2C, PB8 SCL + PB9 SDA, 7-bit addressing |
| I2C timing | `0x00303D5B` in the tested source configuration |
| GPIO | See the wiring table below |

The I2C bus needs SDA/SCL pull-up resistors to 3.3 V; many BME280 breakout boards include them. Confirm the module's voltage requirements before connecting power.

## GPIO configuration

| Pin | CubeMX mode | Pull | Initial level |
| --- | --- | --- | --- |
| PA0 | GPIO_Output, push-pull | None | Low |
| PE0 | GPIO_Output, push-pull | None | Low |
| PC0 | GPIO_Output, push-pull | None | Low |
| PE9 | GPIO_Output, push-pull | None | Low |
| PC2 | GPIO_Output, push-pull | None | Low |
| PC3 | GPIO_Output, push-pull | None | Low |
| PF13 | GPIO_Input | Pull-up | — |
| PF14 | GPIO_Input | Pull-up | — |
| PF15 | GPIO_Output, push-pull | None | Low |
| PB8 | I2C1_SCL | I2C alternate function | — |
| PB9 | I2C1_SDA | I2C alternate function | — |
| PB10 | TIM2_CH3 (set by the application) | None | — |

The application also configures buttons and the heater GPIO at runtime. These `.ioc` settings are included here for completeness and future project regeneration.

## TIM2 configuration

`Fan_PWM_Init()` in `main.c` initializes TIM2 channel 3 and PB10 directly. **Do not call `MX_TIM2_Init()` for TIM2 in the same application.** If your `.ioc` currently generates a TIM2 initialization function, remove the extra invocation from `main()` or disable the duplicate initialization in CubeMX.

Hardware timing:

```text
Timer clock = 16 MHz
Prescaler   = 0
ARR         = 695
PWM mode    = 2
Polarity    = High
Frequency   = 16 MHz / (696) ≈ 22.99 kHz
```

A BC337 open-collector stage inverts PB10, so PWM mode 2 produces the intended duty on the fan's control wire.

## Using the code

1. Generate an STM32CubeIDE project for the NUCLEO-F756ZG.
2. Configure the GPIO and I2C settings above.
3. Check your `Core/Inc/main.h` and HAL project dependencies.
4. Replace `Core/Src/main.c` with this repository's source file.
5. Build in STM32CubeIDE, then flash through ST-LINK.

If you intend to share a fully openable CubeIDE project on GitHub, commit your actual generated `.ioc`, `.project`, `.cproject`, `Core/Inc`, `Drivers`, startup assembly, linker script and any board-support files as well. These were not included in the original code-only export.
