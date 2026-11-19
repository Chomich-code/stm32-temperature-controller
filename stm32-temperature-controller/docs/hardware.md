# Hardware wiring

The design uses an STM32 NUCLEO-F756ZG, a BME280, a 16×2 HD44780-compatible LCD, two push buttons, a four-wire 5 V fan and a resistor heater.

## STM32 connections

| Signal | MCU pin | NUCLEO connector | Mode |
| --- | --- | --- | --- |
| BME280 SCL | PB8 | CN7 pin 2 | I2C1_SCL |
| BME280 SDA | PB9 | CN7 pin 4 | I2C1_SDA |
| LCD RS | PA0 | CN10 pin 29 | GPIO output |
| LCD E | PE0 | CN10 pin 33 | GPIO output |
| LCD D4 | PC0 | CN9 pin 3 | GPIO output |
| LCD D5 | PE9 | CN10 pin 4 | GPIO output |
| LCD D6 | PC2 | CN10 pin 9 | GPIO output |
| LCD D7 | PC3 | CN9 pin 5 | GPIO output |
| Button + | PF13 | CN10 pin 2 | GPIO input + pull-up |
| Button − | PF14 | CN10 pin 8 | GPIO input + pull-up |
| Fan PWM | PB10 | CN10 pin 32 | TIM2_CH3 |
| Heater enable | PF15 | CN10 pin 12 | GPIO output |

## LCD level shifter

The SN74AHCT245N is powered from 5 V and shifts the six MCU LCD signals to the LCD's logic level.

| STM32 signal | SN74 input | SN74 output | LCD pin |
| --- | --- | --- | --- |
| PA0 (RS) | A1, pin 2 | B1, pin 18 | RS, pin 4 |
| PE0 (E) | A2, pin 3 | B2, pin 17 | E, pin 6 |
| PC0 (D4) | A3, pin 4 | B3, pin 16 | D4, pin 11 |
| PE9 (D5) | A4, pin 5 | B4, pin 15 | D5, pin 12 |
| PC2 (D6) | A5, pin 6 | B5, pin 14 | D6, pin 13 |
| PC3 (D7) | A6, pin 7 | B6, pin 13 | D7, pin 14 |

- SN74 pin 20 → 5 V; pin 10 → GND.
- SN74 pin 1 (DIR) → 5 V; pin 19 (/OE) → GND.
- SN74 unused inputs A7/A8 → GND; unused B7/B8 unconnected.
- LCD VSS → GND, VDD → 5 V, RW → GND.
- LCD VO → potentiometer wiper; potentiometer ends → 5 V and GND.
- Use a 100 nF bypass capacitor close to the SN74 supply pins.

## Buttons

Each normally-open button connects its GPIO input to **GND** when pressed. The inputs use internal pull-ups, so no external pull-up resistors are required. Ensure the selected legs are not permanently connected inside the four-leg switch.

## Fan PWM stage

The 5 V fan has separate power and PWM control lines. A BC337 is used as an open-collector pull-down for the PWM input.

- Fan supply positive → external +5 V.
- Fan ground → external GND.
- Fan PWM input → BC337 collector; also pulled up to external +5 V through **10 kΩ**.
- BC337 emitter → common GND.
- PB10 → **1 kΩ** → BC337 base.
- **100 kΩ** from base to emitter.
- Fan tachometer/FG output is not connected.

The BC337 inverts the MCU output, so TIM2 is configured in **PWM mode 2**, active-high polarity. The controller sends 0% to stop the fan, or 40–100% when cooling is active.

## Heater stage

A second BC337 switches a **150 Ω / 0.6 W** resistor on the low side.

- External +5 V → 150 Ω heater resistor → BC337 collector.
- BC337 emitter → common GND.
- PF15 → two **4.7 kΩ resistors in parallel** → BC337 base (2.35 kΩ equivalent).
- **100 kΩ** from base to emitter.

The resistor draws approximately 33 mA and dissipates approximately 0.17 W at 5 V, ignoring small transistor losses. Its surface may still become hot. Mount it away from plastic, wires and the sensor.

Check the specific transistor manufacturer's C/B/E pinout before wiring; do not assume all TO-92 devices have the same lead order.

## Power

- NUCLEO: USB from the computer.
- BME280: 3.3 V logic supply.
- LCD + SN74AHCT245N: 5 V logic supply from the NUCLEO side.
- Fan + heater: independent external 5 V / 1 A supply.
- Both supplies share **GND only**; do not connect their positive 5 V rails together.

The firmware's over-temperature protection measures the BME280, not the heater resistor. Always test the hardware under supervision.
