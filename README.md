# Inverted Pendulum Stepper Motor Control

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/STM32-F401RE-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
[![Motor Driver](https://img.shields.io/badge/Driver-L6474-green)](https://www.st.com/en/motor-drivers/l6474.html)

Real-time inverted pendulum control system using a stepper motor with acceleration control. This project is based on the [STEVAL-EDUKIT01](https://www.st.com/en/evaluation-tools/steval-edukit01.html) educational platform with the original L6474 stepper motor driver.

> **Hardware Platform**: See the [STEVAL-EDUKIT01 product page](https://www.st.com/en/evaluation-tools/steval-edukit01.html) for images and specifications of the rotary inverted pendulum kit.

## Overview

This project implements a 1 kHz control loop for stabilizing an inverted pendulum using a stepper motor. The STM32 microcontroller acts as a low-level real-time controller, receiving acceleration commands from a Raspberry Pi via SPI and providing sensor feedback.

### System Architecture

```
+-------------------------------+
|      Raspberry Pi             |  <- High-level controller (LQR/MPC)
|      (SPI Master)             |
+---------------+---------------+
                | SPI @ 1 kHz
                | (accel cmd down / sensor data up)
+---------------v---------------+
|      NUCLEO-F401RE            |  <- Real-time low-level control
|      + X-NUCLEO-IHM01A1       |     Stepper motor control
+---------------+---------------+
                |
        +-------+-------+
        |               |
   +----v----+    +-----v-----+
   | Stepper |    | Pendulum  |
   |  Motor  |    | Encoder   |
   | (L6474) |    | (2400 CPR)|
   +---------+    +-----------+
```

## Hardware

### Based on STEVAL-EDUKIT01

This project uses the [STEVAL-EDUKIT01](https://www.st.com/en/evaluation-tools/steval-edukit01.html) rotary inverted pendulum kit, which includes:

- Transparent acrylic frame with pendulum arm
- High-precision quadrature encoder (2400 CPR) for pendulum angle
- Stepper motor with L6474 driver
- Sturdy base with motor mount

### Components Used

| Component | Description |
|-----------|-------------|
| **[NUCLEO-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)** | STM32F401RE development board (84 MHz Cortex-M4) |
| **[X-NUCLEO-IHM01A1](https://www.st.com/en/ecosystems/x-nucleo-ihm01a1.html)** | Stepper motor driver expansion board (L6474) |
| **Stepper Motor** | Bipolar stepper (from STEVAL-EDUKIT01) |
| **Raspberry Pi** | High-level controller (any model with SPI) |

### Pin Mapping

| Function | STM32 Pin | Notes |
|----------|-----------|-------|
| Pendulum Encoder A | PA15 | TIM2_CH1 |
| Pendulum Encoder B | PA1 | TIM2_CH2 |
| SPI3 NSS | PA4 | Hardware slave select |
| SPI3 SCK | PC10 | Clock from RPi |
| SPI3 MISO | PC11 | Data to RPi |
| SPI3 MOSI | PC12 | Data from RPi |

## Software

### Features

- **Acceleration Control** mode for direct stepper motor control
- **1 kHz control loop** synchronized with Raspberry Pi
- **SPI slave communication** with DMA (circular buffer)
- **Big-endian protocol** for cross-platform compatibility
- **Safety features**: overcurrent protection, rotor deflection limits

### Control Modes

The project supports different control modes (defined in `main.h`):

| Mode | Description |
|------|-------------|
| `ACCELERATION_CONTROL` | Direct acceleration commands (default, active) |
| `POSITION_CONTROL` | GoTo position commands (commented out) |
| `VELOCITY_CONTROL` | Not implemented |

### SPI Protocol

6-byte bidirectional exchange at 1 kHz (big-endian):

**STM32 -> Raspberry Pi:**
| Bytes | Type | Description |
|-------|------|-------------|
| 0-1 | int16 | Pendulum position (encoder counts) |
| 2-3 | int16 | Stepper motor position (microsteps) |
| 4-5 | int16 | Stepper motor velocity (microsteps/sec) |

**Raspberry Pi -> STM32:**
| Bytes | Type | Description |
|-------|------|-------------|
| 0-1 | int16 | Acceleration command (microsteps/s^2) |
| 2-5 | - | Reserved |

### Building and Flashing

#### Build Commands

```bash
# Configure and build (Debug)
cmake --preset Debug
cmake --build --preset Debug

# Or configure and build (Release)
cmake --preset Release
cmake --build --preset Release

# Clean build
cmake --build build/Debug --target clean
```

#### Flash via OpenOCD

```bash
# Flash Debug build
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_stepper.elf verify reset exit"

# Flash Release build
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Release/invpend_stepper.elf verify reset exit"
```

#### Flash via STM32CubeProgrammer CLI

```bash
# Flash Debug build
STM32_Programmer_CLI -c port=SWD -w build/Debug/invpend_stepper.elf -v -rst

# Flash Release build
STM32_Programmer_CLI -c port=SWD -w build/Release/invpend_stepper.elf -v -rst
```

#### One-liner Build and Flash

```bash
# Build and flash in one command (OpenOCD)
cmake --build --preset Debug && openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_stepper.elf verify reset exit"

# Build and flash in one command (STM32CubeProgrammer)
cmake --build --preset Debug && STM32_Programmer_CLI -c port=SWD -w build/Debug/invpend_stepper.elf -v -rst
```

### Motor Configuration

Key parameters in `main.h`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_SPEED` | 10000 | Maximum speed in microsteps/s |
| `MIN_SPEED` | 30 | Minimum speed in microsteps/s |
| `MAX_ACCEL` | 32767 | Maximum acceleration in microsteps/s^2 |
| `MAX_DECEL` | 32767 | Maximum deceleration in microsteps/s^2 |
| `MAX_TORQUE_CONFIG` | 1200 | Torque regulation current in mA |
| `OVERCURRENT_THRESHOLD` | 2000 | Overcurrent threshold in mA |
| `STEPS_PER_TURN` | 3200 | Microsteps per revolution (1/16 stepping) |

### Dependencies

- [STM32CubeF4](https://www.st.com/en/embedded-software/stm32cubef4.html) HAL drivers
- [X-CUBE-SPN1](https://www.st.com/en/embedded-software/x-cube-spn1.html) L6474 BSP library
- ARM GCC toolchain (arm-none-eabi-gcc)
- CMake 3.22+

## Documentation

Detailed documentation is available in the [`docs/`](docs/) folder:

| Document | Description |
|----------|-------------|
| [claude-instructions.md](docs/claude-instructions.md) | Complete technical reference and wiring guide |

### External Resources

- [STEVAL-EDUKIT01 Product Page](https://www.st.com/en/evaluation-tools/steval-edukit01.html)
- [STEVAL-EDUKIT01 Datasheet (PDF)](https://www.st.com/resource/en/data_brief/steval-edukit01.pdf)
- [STEVAL-EDUKIT01 User Manual (PDF)](https://www.st.com/resource/en/user_manual/um2703-getting-started-with-the-stevaledukit01-rotary-inverted-pendulum-kit-stmicroelectronics.pdf)
- [X-NUCLEO-IHM01A1 User Manual](https://www.st.com/resource/en/user_manual/um1857-getting-started-with-xnucleoihm01a1-stepper-motor-driver-expansion-board-based-on-l6474-for-stm32-nucleo-stmicroelectronics.pdf)
- [L6474 Datasheet](https://www.st.com/resource/en/datasheet/l6474.pdf)

## Getting Started

1. **Hardware Setup**
   - Stack X-NUCLEO-IHM01A1 on NUCLEO-F401RE
   - Connect stepper motor to IHM01A1 motor outputs
   - Connect pendulum encoder to PA15/PA1
   - Wire SPI to Raspberry Pi (see [wiring guide](docs/claude-instructions.md))

2. **Flash Firmware**
   - Build the project using CMake
   - Flash `invpend_stepper.elf` to the NUCLEO board

3. **Run Control Algorithm**
   - Implement your controller on the Raspberry Pi
   - Send acceleration commands via SPI at 1 kHz
   - Read sensor feedback for closed-loop control

## Motor Specifications

| Parameter | Value |
|-----------|-------|
| Motor Type | Bipolar Stepper |
| Driver IC | L6474 |
| Microstepping | 1/16 |
| Steps per Revolution | 3200 (microsteps) |
| Max Current | 1200 mA (configurable) |
| Overcurrent Threshold | 2000 mA |

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [STMicroelectronics](https://www.st.com) for the STEVAL-EDUKIT01 platform and L6474 BSP library
- [Markus Dauberschmidt](https://github.com/OevreFlataeker/steval_edukit_swingup) for swing-up encoder tracking code

---

*For questions or issues, please open a GitHub issue.*
