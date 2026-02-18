# Claude Code Instructions for invpend_stepper

## Project Overview

This is an **STM32F401RE-based inverted pendulum control system** using a stepper motor with L6474 driver. The system runs a 1 kHz control loop that communicates with a Raspberry Pi via SPI to receive acceleration commands and transmit sensor feedback.

### Architecture Summary

```
+-------------------+     SPI (1 kHz)     +-------------------+
|   Raspberry Pi    |<------------------->|   STM32F401RE     |
|   (Controller)    |  accel_cmd / fb     |   (Low-Level)     |
+-------------------+                     +---------+---------+
                                                    |
                    +-------------------------------+-------------------------------+
                    |                                                               |
              +-----v-----+                                                  +------v------+
              |  TIM2     |                                                  |  L6474 BSP  |
              |  Encoder  |                                                  |  Stepper    |
              |  PA15/PA1 |                                                  |  Driver     |
              +-----+-----+                                                  +------+------+
                    |                                                               |
              +-----v-----+                                                  +------v------+
              | Pendulum  |                                                  | Stepper     |
              | (2400 CPR)|                                                  | Motor       |
              +-----------+                                                  +-------------+
```

## Build System

- **Toolchain**: CMake + GCC ARM (arm-none-eabi-gcc)
- **Build directory**: `build/Debug/` or `build/Release/`
- **Target**: `invpend_stepper.elf`

### Build Commands

```bash
# Configure (from project root)
cmake --preset Debug

# Build
cmake --build --preset Debug

# Clean build
cmake --build build/Debug --target clean
```

## Key Source Files

| File | Purpose |
|------|---------|
| [Core/Src/main.c](Core/Src/main.c) | System init, state machine, SPI protocol, encoder reading |
| [Core/Src/l6474_acceleration_control.c](Core/Src/l6474_acceleration_control.c) | Stepper motor acceleration control interface |
| [Core/Src/chrono.c](Core/Src/chrono.c) | DWT-based cycle timing (84 MHz resolution) |
| [Core/Src/stm32f4xx_it.c](Core/Src/stm32f4xx_it.c) | Interrupt handlers |

### Header Files

| File | Key Definitions |
|------|-----------------|
| [Core/Inc/main.h](Core/Inc/main.h) | Control modes, motor params, SPI/UART buffer sizes |
| [Core/Inc/l6474_acceleration_control.h](Core/Inc/l6474_acceleration_control.h) | Acceleration control state structure |
| [Core/Inc/chrono.h](Core/Inc/chrono.h) | `RCC_SYS_CLOCK_FREQ = 84000000` |

### BSP Files (L6474 Driver)

| File | Purpose |
|------|---------|
| [Drivers/BSP/Components/l6474/l6474.c](Drivers/BSP/Components/l6474/l6474.c) | L6474 stepper driver implementation |
| [Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.c](Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.c) | Motor shield interface |
| [Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm01a1_stm32f4xx.c](Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm01a1_stm32f4xx.c) | STM32F4 specific implementation |

## Control Modes

The project supports different control modes (defined in `main.h`):

```c
#define ACCELERATION_CONTROL     // Active - direct acceleration commands
//#define VELOCITY_CONTROL       // NOT IMPLEMENTED
//#define POSITION_CONTROL       // GoTo position commands
```

### Acceleration Control Mode

In acceleration control mode, the Raspberry Pi sends acceleration commands in microsteps/s^2, and the L6474 driver adjusts the step pulse frequency accordingly.

Key functions in `l6474_acceleration_control.c`:

| Function | Description |
|----------|-------------|
| `Init_L6472_Acceleration_Control()` | Initialize acceleration control with motor params |
| `Run_L6472_Acceleration_Control()` | Apply acceleration command (called every 1ms) |
| `StepClockHandler_L6472_Acceleration_Control()` | ISR callback for step pulse generation |
| `PostProcess_StepClockHandler_L6472_Acceleration_Control()` | Update PWM period after ISR |
| `GetPosition_L6472_Acceleration_Control()` | Read motor position (involves SPI) |
| `GetVelocity_L6472_Acceleration_Control()` | Read motor velocity |

## Hardware Wiring Guide

### 1. Pendulum Encoder Wiring (2400 CPR)

Connect the pendulum quadrature encoder to TIM2:

| Encoder Wire | STM32 Pin | Notes |
|--------------|-----------|-------|
| **Channel A** | PA15 | TIM2_CH1 (Morpho CN7-17) |
| **Channel B** | PA1 | TIM2_CH2 |
| **VCC** | 5V or 3.3V | Depends on encoder |
| **GND** | GND | |

### 2. Raspberry Pi SPI Wiring

Connect Raspberry Pi GPIO to STM32 SPI3 (slave mode):

| RPi Pin | RPi GPIO | Signal | STM32 Pin | Morpho |
|---------|----------|--------|-----------|--------|
| 24 | GPIO8 (CE0) | NSS | PA4 | CN7-32 |
| 23 | GPIO11 (SCLK) | SCK | PC10 | CN7-1 |
| 21 | GPIO9 (MISO) | MISO | PC11 | CN7-2 |
| 19 | GPIO10 (MOSI) | MOSI | PC12 | CN7-3 |
| 6 | - | GND | GND | CN7-20 |

**SPI Configuration on Raspberry Pi**:
```python
# Python spidev example
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0b00  # CPOL=0, CPHA=0
```

**Important**: Connect GND between RPi and STM32!

### 3. X-NUCLEO-IHM01A1 Motor Shield

The IHM01A1 shield stacks directly onto the NUCLEO-F401RE. Key connections handled by shield:

| Function | Description |
|----------|-------------|
| L6474 SPI | SPI1 (internal to shield) |
| PWM Step Clock | TIM3_CH1 (step pulse generation) |
| Direction | GPIO controlled by BSP |
| Motor Outputs | A+, A-, B+, B- on J1/J2 |

**Motor Connection (J1/J2 on IHM01A1)**:

| Terminal | Connection |
|----------|------------|
| A+ / A- | Stepper motor coil A |
| B+ / B- | Stepper motor coil B |

**Power Supply (J3 on IHM01A1)**:
- VIN: 8-45V DC (12V typical for this motor)
- GND: Power ground

### 4. UART Debug Connection

UART2 is connected to ST-LINK virtual COM port (no external wiring needed):

| Function | Pin | Notes |
|----------|-----|-------|
| TX | PA2 | To PC via ST-LINK |
| RX | PA3 | From PC via ST-LINK |

**Baud rate**: 115200 (8N1)

### Wiring Checklist

- [ ] **Pendulum encoder**: A->PA15, B->PA1, VCC->5V, GND->GND
- [ ] **Stepper motor**: Connected to IHM01A1 J1/J2 terminals
- [ ] **RPi SPI**: NSS->PA4, SCK->PC10, MISO->PC11, MOSI->PC12, GND->GND
- [ ] **X-NUCLEO-IHM01A1**: Stacked on Nucleo
- [ ] **Power**: 12V DC to IHM01A1 J3 terminals
- [ ] **Common ground**: Ensure RPi, Nucleo, and power supply share GND

## SPI Protocol

**Buffer size**: 6 bytes, **Endianness**: Big-endian

### RX from Raspberry Pi
```
Bytes [0-1]: int16_t accel_cmd       // Acceleration command in microsteps/s^2
Bytes [2-3]: int16_t recv_number2    // Reserved
Bytes [4-5]: int16_t recv_number3    // Reserved
```

### TX to Raspberry Pi
```
Bytes [0-1]: int16_t encoder_pos     // Pendulum encoder position (counts)
Bytes [2-3]: int16_t rotor_pos       // Motor position (microsteps)
Bytes [4-5]: int16_t rotor_vel       // Motor velocity (microsteps/sec)
```

## State Machine (`main.c` while loop)

```
STATE 0 (START)    -> Wait for first SPI transaction
       |
       v
STATE 1 (READ)     -> Read encoders, prepare TX buffer
       |
       v
STATE 2 (WAIT_SPI) -> Wait for SPI completion
       |
       v (timeout)
STATE 3 (OVERTIME) -> Handle overtime, keep reading rotor position
       |
       v
STATE 4 (CONTROL)  -> Apply acceleration command via L6474
       |
       +---------> [loop back to STATE 1]

STATE 50 (ERROR)   -> Rotor out of safe range, hard stop
STATE 99 (HALT)    -> Halted state
```

### Safety Limits

```c
#define MAX_DEFLECTION_REV 1  // Maximum deflection in revolutions before hard stop
```

If the rotor exceeds `MAX_DEFLECTION_REV * STEPS_PER_TURN` (3200 microsteps), the motor hard stops.

## Motor Configuration

L6474 initialization parameters (in `main.c`):

```c
L6474_Init_t gL6474InitParams = {
    MAX_ACCEL,               // Acceleration rate in step/s^2 (32767)
    MAX_DECEL,               // Deceleration rate in step/s^2 (32767)
    MAX_SPEED,               // Maximum speed in step/s (10000)
    MIN_SPEED,               // Minimum speed in step/s (30)
    MAX_TORQUE_CONFIG,       // Torque regulation current in mA (1200)
    OVERCURRENT_THRESHOLD,   // Overcurrent threshold in mA (2000)
    L6474_CONFIG_OC_SD_ENABLE,         // Overcurrent shutdown enabled
    L6474_CONFIG_EN_TQREG_TVAL_USED,   // Torque regulation method
    L6474_STEP_SEL_1_16,               // 1/16 microstepping
    L6474_SYNC_SEL_1_2,                // Sync selection
    L6474_FAST_STEP_12us,              // Fall time
    L6474_TOFF_FAST_8us,               // Max fast decay time
    3,                                 // Min ON time (us)
    21,                                // Min OFF time (us)
    L6474_CONFIG_TOFF_044us,           // Target switching period
    L6474_CONFIG_SR_320V_us,           // Slew rate
    L6474_CONFIG_INT_16MHZ,            // Clock setting
    (L6474_ALARM_EN_OVERCURRENT | ...)  // Alarm enables
};
```

## Timing & Interrupts

| Component | Frequency | Priority | Notes |
|-----------|-----------|----------|-------|
| System Clock | 84 MHz | - | HSI + PLL |
| Step Clock (TIM PWM) | Variable | Depends on speed | Step pulse generation |
| SPI3 DMA | RPi-driven (~1 kHz) | 0 | Highest priority |
| Control Loop | 1 kHz target | main loop | |

## Critical Constants

```c
// Timing
T_SAMPLE            = 0.001f    // 1 kHz sample time

// Encoder
COUNTS_PER_TURN     = 2400      // Pendulum encoder counts per revolution

// Stepper Motor
STEPS_PER_TURN      = 3200      // Microsteps per revolution (1/16 stepping)
MAX_SPEED           = 10000     // Maximum speed in microsteps/s
MIN_SPEED           = 30        // Minimum speed in microsteps/s
MAX_ACCEL           = 32767     // Maximum acceleration in microsteps/s^2
MAX_TORQUE_CONFIG   = 1200      // Torque current in mA
OVERCURRENT_THRESHOLD = 2000    // OCD threshold in mA

// Buffer sizes
SPI_BUFFER_SIZE     = 6         // 6 bytes = 3x int16
UART_BUFFER_SIZE    = 150
```

## Development Workflow

### Adding New Functionality

1. Add source files to `Core/Src/`, headers to `Core/Inc/`
2. Update `cmake/stm32cubemx/CMakeLists.txt` if needed
3. Include headers in `main.c` within `USER CODE BEGIN Includes`
4. Initialize in `USER CODE BEGIN SysInit` section
5. Call from main loop

### Key Integration Points

```c
// In main.c USER CODE BEGIN SysInit:
BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
Init_L6472_Acceleration_Control(&gL6474InitParams, T_SAMPLE);
Chrono_Init();

// In main.c while loop:
// State machine runs, calls Run_L6472_Acceleration_Control(accel_cmd)
PostProcess_StepClockHandler_L6472_Acceleration_Control();
```

## Common Issues

| Issue | Solution |
|-------|----------|
| SPI not working | Check NSS is hardware mode, DMA streams configured |
| Encoder not counting | Verify TIM2 encoder mode, check wiring |
| Motor not responding | Check L6474 initialization, verify power supply |
| Timing drift | Verify 84 MHz clock, check `Chrono_Init()` |
| Motor stalls | Check current limit, may need higher `MAX_TORQUE_CONFIG` |
| Overcurrent fault | Reduce acceleration, check motor wiring |
| Step pulse glitches | Use `PostProcess_StepClockHandler` for PWM updates |

### L6474 Status Flags

The `MyFlagInterruptHandler()` in `main.c` handles various L6474 status flags:

| Flag | Description |
|------|-------------|
| `L6474_STATUS_HIZ` | Power bridges disabled |
| `L6474_STATUS_NOTPERF_CMD` | Command can't be performed (often during HIZ) |
| `L6474_STATUS_WRONG_CMD` | Unknown SPI command |
| `L6474_STATUS_UVLO` | Undervoltage lock-out |
| `L6474_STATUS_TH_WRN` | Thermal warning |
| `L6474_STATUS_TH_SD` | Thermal shutdown |
| `L6474_STATUS_OCD` | Overcurrent detection |

## Build and Flash

```bash
# Build
cmake --build --preset Debug

# Flash via OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_stepper.elf verify reset exit"

# Or flash via STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -w build/Debug/invpend_stepper.elf -v -rst
```

## Memory Usage

Typical Debug build:
```
Memory region         Used Size  Region Size  %age Used
             RAM:        3560 B        96 KB      3.62%
           FLASH:       41100 B       512 KB      7.84%
```

## Dependencies

- **STM32 HAL**: F4 series HAL drivers
- **L6474 BSP**: X-CUBE-SPN1 stepper driver library
- **Motor Shield**: X-NUCLEO-IHM01A1

## Differences from BLDC Project

| Aspect | BLDC Project | Stepper Project |
|--------|--------------|-----------------|
| Motor Type | BLDC (FOC) | Stepper (L6474) |
| Control SDK | MC SDK v6.4.1 | L6474 BSP |
| Control Input | Torque (mNm) | Acceleration (steps/s^2) |
| SPI Buffer | 10 bytes | 6 bytes |
| Motor Encoder | Yes (differential) | No (open loop) |
| PWM Frequency | 16 kHz (FOC) | Variable (step clock) |
