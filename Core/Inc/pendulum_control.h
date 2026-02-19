/**
  ******************************************************************************
  * @file    pendulum_control.h
  * @brief   Inverted pendulum control state machine and encoder interface
  *
  * Implements:
  * - SPI slave communication with Raspberry Pi (8-byte protocol)
  * - Pendulum encoder reading on TIM2
  * - Stepper motor control via L6474 driver
  * - 1 kHz control loop state machine
  ******************************************************************************
  */

#ifndef PENDULUM_CONTROL_H
#define PENDULUM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "app_config.h"

/*******************************************************************************
 * State Machine Definitions
 ******************************************************************************/

/** @defgroup StateMachine_States State Machine States
  * @{
  */
typedef enum {
    STATE_START = 0,        /**< Wait for first SPI transaction */
    STATE_READ = 1,         /**< Read encoders and prepare TX data */
    STATE_WAIT_SPI = 2,     /**< Wait for SPI transaction completion */
    STATE_OVERTIME = 3,     /**< Handle SPI timeout */
    STATE_CONTROL = 4,      /**< Apply acceleration command */
    STATE_ERROR_ROTOR = 50, /**< Error state - rotor out of safe range */
    STATE_HALT = 99         /**< Halted state */
} MainState_t;
/** @} */

/*******************************************************************************
 * SPI Protocol Definitions
 ******************************************************************************/

/**
  * SPI Protocol (8 bytes, big-endian):
  *
  * RX from RPi (Master):
  *   [0-1] int16_t accel_cmd       - Acceleration command in steps/s^2
  *   [2-7] reserved                - Reserved for future use
  *
  * TX to RPi (Master):
  *   [0-1] int16_t pendulum_pos    - Pendulum encoder position (counts)
  *   [2-3] int16_t pendulum_vel    - Pendulum velocity (counts/sec / DIV)
  *   [4-5] int16_t rotor_pos       - Rotor position (microsteps)
  *   [6-7] int16_t rotor_vel       - Rotor velocity (microsteps/sec)
  */

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
  * @brief  Velocity calculator state structure
  * @note   Used for pendulum velocity calculation with EMA filter
  */
typedef struct {
    uint32_t enc_prev;          /**< Previous encoder count */
    uint32_t time_prev_cycles;  /**< Previous DWT cycle count */
    float vel_filtered;         /**< Filtered velocity (counts/sec) */
    float filter_alpha;         /**< EMA filter coefficient */
    uint16_t cpr;               /**< Counts per revolution (for wraparound) */
    uint8_t wrap_at_cpr;        /**< 1 if wraps at CPR, 0 if 32-bit wrap */
} VelocityCalc_t;

/**
  * @brief  Pendulum encoder state structure
  * @note   Uses TIM2 which is a 32-bit timer
  */
typedef struct {
    uint32_t cnt;               /**< Current counter value (32-bit for TIM2) */
    uint32_t previous_cnt;      /**< Previous counter value */
    int32_t position_steps;     /**< Accumulated position in encoder steps */
    int32_t position_init;      /**< Initial position offset */
    uint16_t counts_per_turn;   /**< Encoder counts per revolution */
} Pendulum_Encoder_TypeDef;

/*******************************************************************************
 * External Variables (defined in pendulum_control.c)
 ******************************************************************************/

/** Current state machine state */
extern volatile MainState_t g_state;

/** SPI transaction complete flag */
extern volatile uint8_t spi_txrx_flag;

/** SPI error flag */
extern volatile uint8_t spi_err_flag;

/** UART transmit complete flag */
extern volatile uint8_t uart_xmit_flag;

/** UART error flag */
extern volatile uint8_t uart_err_flag;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
  * @brief  Initialize pendulum control subsystem
  *         - Initializes chrono timer
  *         - Initializes encoder structures
  *         - Resets state machine to STATE_START
  */
void Pendulum_Init(void);

/**
  * @brief  Read pendulum encoder position
  * @param  enc: Pointer to encoder structure
  * @param  htim: Pointer to TIM handle (TIM2)
  */
void Pendulum_Encoder_Read(Pendulum_Encoder_TypeDef *enc, TIM_HandleTypeDef *htim);

/**
  * @brief  Start SPI DMA circular communication
  *         Call this once after initialization
  */
void SPI_StartCommunication(void);

/**
  * @brief  Run state machine iteration
  *         Call this continuously from main loop
  *         Handles: encoder reading, SPI data exchange, motor control
  */
void StateMachine_Run(void);

/**
  * @brief  Get current acceleration command value
  * @retval Acceleration command in steps/s^2
  */
int16_t GetAccelCommand(void);

/**
  * @brief  Get current pendulum position
  * @retval Pendulum position in encoder counts
  */
int32_t GetPendulumPosition(void);

/**
  * @brief  Get current rotor (stepper) position
  * @retval Rotor position in microsteps
  */
int32_t GetRotorPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* PENDULUM_CONTROL_H */
