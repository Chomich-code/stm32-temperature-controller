# STM32 Temperature Controller

A breadboard temperature controller built around the **STM32 NUCLEO-F756ZG**. It measures temperature with a BME280, heats with a small resistor heater, and controls a 5 V fan using a PID-driven PWM signal.

The setpoint is adjusted with two buttons and the current state is shown on a 16×2 character LCD.

## Features

- BME280 temperature measurements over I²C
- 16×2 LCD in 4-bit mode via an SN74AHCT245N level shifter
- Setpoint adjustment in 0.5 °C steps (10–40 °C)
- On/off heating using a 150 Ω resistor and an NPN transistor
- PID fan control through TIM2 channel 3 (~23 kHz PWM)
- Separate heating, cooling and idle states
- Sensor fault handling, heater cutoff and emergency stop

## Control behavior

The default setpoint is **25 °C**.

| Condition | Action |
| --- | --- |
| Temperature ≤ 24 °C while idle | Start heating |
| Temperature reaches 25 °C while heating | Stop heating |
| Temperature ≥ 26 °C while idle | Start cooling |
| Temperature reaches 25 °C while cooling | Stop cooling |
| Neither threshold is crossed | Remain idle |

The ±1 °C values are **start thresholds around the setpoint**, not a symmetric ±1 °C switching hysteresis. Once a mode starts, it stays active until the setpoint is reached. A three-second pause follows an active mode before another mode can start.

The heater is switched on or off. The fan runs at **40–100% duty cycle** during cooling; the PID loop adjusts duty from the measured temperature. An inactive fan receives 0% PWM.

> A normal fan can remove heat, but it cannot actively cool the measured air below the incoming ambient air temperature.

## Hardware

| Component | Connection |
| --- | --- |
| BME280 SCL / SDA | PB8 / PB9 (I2C1) |
| LCD RS, E, D4–D7 | PA0, PE0, PC0, PE9, PC2, PC3 |
| Button + / Button − | PF13 / PF14 (active low) |
| Fan PWM output | PB10 (TIM2_CH3) |
| Heater control | PF15 |

See [hardware wiring](docs/hardware.md) for the complete wiring table and power connections.

## Firmware

The firmware is written in C using **STM32CubeIDE** and **STM32 HAL**. The clock uses the 16 MHz HSI oscillator. The application configures TIM2 and PB10 directly; do not initialize TIM2 a second time through generated CubeMX code.

```text
Core/Src/main.c           Application source
README.md                 Project overview
docs/hardware.md          Pinout, transistors and power
docs/firmware.md          Control logic and parameters
docs/stm32cubeide.md      STM32CubeIDE setup
```

### Build and flash

**This is a source-code repository, not a complete standalone STM32CubeIDE workspace.** The original `.ioc`, `main.h`, startup file, linker script and HAL/CMSIS files were not part of the source package used here. To build, use your existing working `PID_REGULATOR` project or add those generated files from your local STM32CubeIDE workspace.

1. Open your existing `PID_REGULATOR` project in STM32CubeIDE.
2. Back up its current `Core/Src/main.c`.
3. Copy this repository's `Core/Src/main.c` into the same path.
4. Verify the clock, I2C1 and GPIO settings in [STM32CubeIDE setup](docs/stm32cubeide.md).
5. Make sure `MX_TIM2_Init()` is **not** also called; PWM is configured in `Fan_PWM_Init()`.
6. Build the project and flash it using ST-LINK.

For a fully reproducible GitHub repository, add your `.ioc`, `Core/Inc`, `Drivers`, `startup` file, linker script and the CubeIDE project metadata from your working local project. Keep build output out of Git using the included `.gitignore`.

### Main settings

The primary configuration values are near the top of `Core/Src/main.c`:

```c
#define SETPOINT_DEFAULT_X2  50    // 25.0 C
#define HEAT_ON_DELTA_C       1.0f
#define COOL_ON_DELTA_C       1.0f
#define FAN_MIN_ACTIVE_PCT   40U
#define PID_KP               15.0f
#define PID_KI                0.08f
#define PID_KD                6.0f
```

Set `FAN_USE_PID` to `0U` to test a fixed fan duty cycle using `FAN_FIXED_PWM_PCT`. Normal operation uses `FAN_USE_PID = 1U`.

See [firmware notes](docs/firmware.md) for the state machine, PID implementation and safety limits.

## Using the controller

- **+ button:** increase setpoint by 0.5 °C.
- **− button:** decrease setpoint by 0.5 °C.
- **Both buttons for 1.2 seconds:** latch emergency stop until reset.
- **LCD line 1:** measured and requested temperatures (`T` and `S`).
- **LCD line 2:** heating, cooling duty cycle, standby or fault.

## Testing

The hardware was tested incrementally: BME280 communication and LCD output, button input, fan duty changes at 40% and 70%, fan stop at 0%, and resistor heating. The integrated heating/cooling controller was also run on the breadboard.

No calibrated temperature response, settling-time or overshoot measurements have been recorded. PID gains are starting values, not experimentally optimized coefficients.

## Safety

The fan and heater use a separate **5 V / 1 A supply**. The STM32 and power stage share ground, but the external supply's +5 V line must **not** be tied to the NUCLEO USB-derived +5 V line.

Keep the heater resistor away from the breadboard plastic, wiring and the BME280 package. The BME280 heater cutoff at 35 °C monitors the sensor's local temperature, **not the resistor itself**. This is a supervised educational prototype, not a safety-rated thermal controller.
