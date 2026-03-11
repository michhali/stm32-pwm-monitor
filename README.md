# STM32 PWM Monitor and Control System

Embedded systems project built on the STM32F051 microcontroller to monitor and control PWM-related signals using ADC, DAC, EXTI interrupts, timer-based frequency measurement, and SPI-driven OLED output.

## Overview

This project implements a real-time embedded system that reads an analog input from a potentiometer, converts it to a digital value using the ADC, and forwards that value through the DAC to drive an optocoupler. The optocoupler affects the timing resistance of a 555 timer circuit, allowing the microcontroller to influence the output frequency.

The system can measure frequency from two different sources:

- a function generator input
- a 555 timer output

A pushbutton interrupt switches between measurement modes, and the OLED display shows:

- measured resistance
- active signal source
- measured frequency

## Hardware

- STM32F051 microcontroller
- STM32F0 Discovery board
- PBMCUSLK board
- NE555 timer
- 4N35 optocoupler
- potentiometer
- SSD1306 OLED display

## Technical Highlights

- Configured the STM32 system clock to 48 MHz using the PLL
- Used ADC on PA1 to sample the potentiometer input
- Used DAC on PA4 to drive the optocoupler control voltage
- Measured signal frequency using TIM2 as a 32-bit timer
- Implemented EXTI-based rising edge capture for frequency measurement
- Added interrupt-driven mode switching between PB2 and PB3
- Built SPI communication for OLED rendering
- Corrected OLED orientation and column alignment in software
- Added software debouncing for pushbutton mode switching

## Architecture

### Input Path
Potentiometer → ADC (PA1) → digital value

### Control Path
ADC value → DAC (PA4) → 4N35 optocoupler → 555 timer timing resistance

### Measurement Path
- PB2: function generator input
- PB3: 555 timer output
- TIM2 + EXTI rising-edge capture for frequency calculation

### Output Path
SPI → SSD1306 OLED display

## Results

The system successfully:

- measured the potentiometer across the full input range
- output a matching DAC control voltage
- measured function generator inputs from 100 Hz to 10 kHz with about ±1% accuracy
- measured 555 timer output roughly in the 700 to 1200 Hz range with about ±1 to ±2% accuracy
- switched cleanly between signal sources
- displayed resistance, signal source, and frequency in real time

## Engineering Notes

A calibrated timer frequency constant was introduced to compensate for internal oscillator error. The implementation also used:

- internal pull-down resistors for stable digital inputs
- a free-running 32-bit TIM2 counter
- EXTI line masking and unmasking to prevent stale interrupts
- display offset correction for proper OLED text alignment

## Limitations

- frequency display can flicker due to lack of averaging/filtering
- optocoupler response is nonlinear at low LED currents
- blocking SPI writes limit faster display refresh
- noisy input edges can affect measurement stability

## Repository Structure

```text
src/       source code
docs/      report, schematic, images
README.md  project overview
