/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Application configuration and debug settings
  *
  * Central configuration file for compile-time options, debug flags, and
  * test modes. Modify these defines to enable/disable features.
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Debug and Test Configuration
 ******************************************************************************/

/**
  * @brief  Test mode - disable motor power stage
  *
  * When set to 1, the stepper motor is completely disabled:
  * - No step pulses are generated
  * - All acceleration commands are ignored (treated as 0)
  * - Motor will NOT move even if SPI receives a command
  *
  * Use this mode when testing encoder and SPI communication without
  * risking unintended motor movement. Set to 0 for normal operation.
  */
#define TEST_MODE_NO_MOTOR          0

/**
  * @brief  Skip waiting for SPI master
  *
  * When set to 1, the state machine runs independently without waiting for
  * SPI transactions from the Raspberry Pi. Useful for testing encoder
  * functionality without the Pi connected.
  *
  * Set to 0 to enable normal SPI-synchronized operation.
  */
#define SKIP_SPI_WAIT               0

/**
  * @brief  Enable pendulum encoder debug output via UART
  *
  * When set to 1, the pendulum encoder count is periodically printed to UART2
  * (ST-Link virtual COM port). This is useful for testing the encoder
  * connection without needing the Raspberry Pi or motor control.
  *
  * Set to 0 for normal operation (no debug output).
  */
#define DEBUG_PENDULUM_ENCODER      0

/**
  * @brief  Enable stepper motor debug output via UART
  *
  * When set to 1, stepper motor state, position, and velocity are printed
  * to UART2 periodically. Useful for diagnosing motor control issues.
  *
  * Set to 0 for normal operation (no debug output).
  */
#define DEBUG_STEPPER               0

/**
  * @brief  Debug print interval in milliseconds
  *
  * Controls how often debug information is printed when DEBUG_PENDULUM_ENCODER
  * or DEBUG_STEPPER is enabled. Default is 100ms (10 Hz).
  */
#define DEBUG_PRINT_INTERVAL_MS     100

/**
  * @brief  Button-triggered test mode
  *
  * When set to 1, pressing the blue user button (PC13) applies a fixed
  * acceleration for a set duration, overriding any SPI commands.
  * Useful for testing motor movement without requiring RPi control.
  *
  * Set to 0 for normal SPI-controlled operation.
  */
#define TEST_MODE_BUTTON            0

/*******************************************************************************
 * Control Mode Selection
 ******************************************************************************/

/**
  * @brief  Enable acceleration control mode (default)
  *
  * Stepper motor receives acceleration commands from SPI master.
  * Comment out to disable.
  */
#define ACCELERATION_CONTROL

/**
  * @brief  Enable position control mode
  *
  * Stepper motor receives position setpoints from SPI master.
  * Uncomment to enable (mutually exclusive with ACCELERATION_CONTROL).
  */
//#define POSITION_CONTROL

/**
  * @brief  Enable velocity control mode (NOT IMPLEMENTED)
  *
  * Reserved for future implementation.
  */
//#define VELOCITY_CONTROL

/*******************************************************************************
 * Timing Configuration
 ******************************************************************************/

/**
  * @brief  Control loop sample time in seconds
  *
  * Should match the sample time in the MATLAB/Simulink model on the RPi.
  */
#define T_SAMPLE                    0.001f

/**
  * @brief  Overtime detection multiplier
  *
  * If SPI transaction takes longer than T_SAMPLE * OVERTIME_FACTOR,
  * the system enters overtime handling state.
  */
#define OVERTIME_FACTOR             1.5f

/**
  * @brief  Rotor position check interval during overtime (seconds)
  *
  * In overtime state, rotor position is checked at this interval
  * to detect overrange conditions.
  */
#define ROTOR_CHECK_INTERVAL_S      0.05f

/**
  * @brief  Stepper position read decimation
  *
  * Stepper position is read every N control cycles.
  * Default: 5 (read position every 5ms at 1kHz control rate)
  */
#define STEPPER_POSITION_DECIMATION 5

/*******************************************************************************
 * Velocity Calculation
 ******************************************************************************/

/**
  * @brief  Pendulum velocity filter coefficient (exponential moving average)
  *
  * Range: 0.01 to 1.0
  * - Lower values (0.01-0.1): More filtering, slower response, less noise
  * - Higher values (0.5-1.0): Less filtering, faster response, more noise
  *
  * Default: 0.1 (smooth output, ~10 sample time constant)
  */
#define PEND_VEL_FILTER_ALPHA       0.1f

/**
  * @brief  Pendulum velocity output resolution divisor
  *
  * The internal velocity (counts/second) is divided by this factor before
  * sending over SPI. This reduces resolution but increases range.
  *
  * Example: With divisor=1, internal velocity of 1000 counts/s outputs as 1000
  */
#define PEND_VEL_RESOLUTION_DIV     1

/*******************************************************************************
 * Communication Configuration
 ******************************************************************************/

/**
  * @brief  SPI buffer size in bytes
  *
  * Buffer for SPI DMA transfers (4x int16_t = 8 bytes)
  * TX: pend_pos, pend_vel, rotor_pos, rotor_vel
  * RX: accel_cmd, reserved...
  */
#define SPI_BUFFER_SIZE             8

/**
  * @brief  UART buffer size in bytes
  *
  * Buffer for debug UART transmissions.
  */
#define UART_BUFFER_SIZE            150

/**
  * @brief  UART debug print decimation
  *
  * Only print every N cycles when continuous debug output is enabled.
  */
#define UART_DECIMATION             2000

/*******************************************************************************
 * Encoder Configuration
 ******************************************************************************/

/**
  * @brief  Pendulum encoder counts per revolution
  *
  * Quadrature encoder resolution (4x counts per encoder line).
  */
#define COUNTS_PER_TURN             2400

/*******************************************************************************
 * Stepper Motor Configuration
 ******************************************************************************/

/**
  * @brief  Microstepping resolution
  *
  * Microsteps per full step. Configure in gL6474InitParams (STEP_SEL field).
  * With 1/16 microstepping and 200 steps/rev motor: 3200 microsteps/rev
  */
#define STEPS_PER_TURN              3200

/**
  * @brief  Minimum motor current in mA
  */
#define MIN_CURRENT                 100

/**
  * @brief  Maximum motor current in mA
  */
#define MAX_CURRENT                 800

/**
  * @brief  Maximum motor speed in microsteps/second
  */
#define MAX_SPEED                   10000

/**
  * @brief  Minimum motor speed in microsteps/second
  */
#define MIN_SPEED                   30

/**
  * @brief  Maximum acceleration in microsteps/second^2
  */
#define MAX_ACCEL                   32767

/**
  * @brief  Maximum deceleration in microsteps/second^2
  */
#define MAX_DECEL                   32767

/**
  * @brief  Torque regulation current in mA (TVAL register)
  *
  * Range: 31.25mA to 4000mA
  */
#define MAX_TORQUE_CONFIG           1200

/**
  * @brief  Overcurrent detection threshold in mA (OCD_TH register)
  *
  * Range: 375mA to 6000mA
  */
#define OVERCURRENT_THRESHOLD       2000

/*******************************************************************************
 * Safety Configuration
 ******************************************************************************/

/**
  * @brief  Maximum rotor deflection in revolutions
  *
  * If the rotor exceeds this deflection from home position,
  * the motor is stopped and system enters error state.
  */
#define MAX_DEFLECTION_REV          1.0f

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
