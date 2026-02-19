/**
  ******************************************************************************
  * @file    pendulum_control.c
  * @brief   Inverted pendulum control state machine implementation
  *
  * Implements the 1 kHz control loop state machine that:
  * - Reads pendulum encoder position from TIM2
  * - Reads stepper motor position/velocity from L6474 driver
  * - Exchanges data with Raspberry Pi via SPI3 slave
  * - Applies acceleration commands to stepper motor
  ******************************************************************************
  */

#include "pendulum_control.h"
#include "l6474_acceleration_control.h"
#include "chrono.h"
#include "app_config.h"
#include "x_nucleo_ihmxx.h"
#include "l6474.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

/*******************************************************************************
 * External Handles (defined in main.c by CubeMX)
 ******************************************************************************/
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

/*******************************************************************************
 * Module Variables
 ******************************************************************************/

/** Current state machine state */
volatile MainState_t g_state = STATE_START;

/** SPI transaction complete flag (set in DMA callback) */
volatile uint8_t spi_txrx_flag = 0;

/** SPI error flag */
volatile uint8_t spi_err_flag = 0;

/** UART transmit complete flag */
volatile uint8_t uart_xmit_flag = 1;

/** UART error flag */
volatile uint8_t uart_err_flag = 0;

/** SPI receive buffer */
static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];

/** SPI transmit buffer */
static uint8_t spi_tx_buf[SPI_BUFFER_SIZE];

/** Received values from SPI (big-endian converted) */
static int16_t recv_accel_cmd = 0;    /**< Acceleration command in steps/s^2 */

/** Values to send via SPI (big-endian converted) */
static int16_t send_pend_pos = 0;     /**< Pendulum position (counts) */
static int16_t send_pend_vel = 0;     /**< Pendulum velocity (counts/sec / DIV) */
static int16_t send_rotor_pos = 0;    /**< Rotor position (microsteps) */
static int16_t send_rotor_vel = 0;    /**< Rotor velocity (microsteps/sec) */

/** Current acceleration command */
static int16_t accel_cmd = 0;

/** Encoder position (accumulated) */
static int32_t encoder_pos = 0;

/** Rotor (stepper) position in microsteps */
static int32_t rotor_pos = 0;

/** Rotor velocity (direct from L6474 driver, open-loop) */
static int16_t rotor_vel = 0;

/** Rotor position read decimation counter */
static uint16_t rotor_pos_cnt = 0;

/** Pendulum encoder instance */
static Pendulum_Encoder_TypeDef pendulum_enc = {
    .cnt = 0,
    .previous_cnt = 0,
    .position_steps = 0,
    .position_init = 0,
    .counts_per_turn = COUNTS_PER_TURN
};

/** Pendulum velocity calculator (with EMA filter) */
static VelocityCalc_t pend_vel_calc = {
    .enc_prev = 0,
    .time_prev_cycles = 0,
    .vel_filtered = 0.0f,
    .filter_alpha = PEND_VEL_FILTER_ALPHA,
    .cpr = COUNTS_PER_TURN,
    .wrap_at_cpr = 0  /* 32-bit timer, no CPR wrap */
};

/** Cycle timer for timing measurements */
static Chrono_TypeDef cycle_timer = {0, 0, 0.0f};

#if DEBUG_PENDULUM_ENCODER || DEBUG_STEPPER
/** UART transmit buffer for debug messages */
static char uart_tx_buf[UART_BUFFER_SIZE];
#endif

/** Error and overtime counters */
static uint32_t err_cnt = 0;
static uint32_t overtime_cnt = 0;
static uint32_t cycle_cnt = 0;

#if TEST_MODE_BUTTON
/** Button test mode variables */
static volatile uint8_t button_test_active = 0;
static uint32_t button_test_start_tick = 0;
#define BUTTON_TEST_DURATION_MS  2000
#define BUTTON_TEST_ACCEL_VALUE  5000
#else
/** Rotor fault flag - set when rotor exceeds safe range */
static volatile uint8_t rotor_fault_active = 0;

/** Fault recovery phase:
 *  0 = fault active, waiting for first button press
 *  1 = motor disabled, waiting for second button press
 */
static volatile uint8_t fault_recovery_phase = 0;

/** Button press request flags (set in ISR, handled in main loop) */
static volatile uint8_t button_press_pending = 0;
#endif

#if DEBUG_PENDULUM_ENCODER || DEBUG_STEPPER
/** Debug print timer */
static Chrono_TypeDef debug_timer = {0, 0, 0.0f};
#define DEBUG_PRINT_INTERVAL_S  (DEBUG_PRINT_INTERVAL_MS / 1000.0f)
#endif

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static inline int16_t swap16(int16_t v);
static void UART_Print_Quick(const char *msg);
static int32_t CalcVelocity(VelocityCalc_t *vc, uint32_t cnt);
#if !TEST_MODE_BUTTON
static void ProcessButtonPress(void);
#endif

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
  * @brief  Swap endianness of 16-bit value (for big-endian SPI protocol)
  * @param  v: Value to swap
  * @retval Byte-swapped value
  */
static inline int16_t swap16(int16_t v)
{
    return (int16_t)(((uint16_t)v >> 8) | ((uint16_t)v << 8));
}

/**
  * @brief  Quick UART print helper with short timeout
  * @param  msg: Null-terminated string to transmit
  */
static void UART_Print_Quick(const char *msg)
{
    if (msg) {
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 1);
    }
}

/**
  * @brief  Saturating cast from int32_t to int16_t
  */
static inline int16_t saturate_int16(int32_t val)
{
    if (val > INT16_MAX) return INT16_MAX;
    if (val < INT16_MIN) return INT16_MIN;
    return (int16_t)val;
}

/**
  * @brief  Calculate velocity with EMA filtering
  *
  * Uses floating-point math and exponential moving average for smooth output.
  * Uses DWT cycle counter for precise timing measurement.
  *
  * @param  vc: Pointer to velocity calculator state
  * @param  cnt: Current encoder count
  * @retval Velocity in counts/second (int32_t for full precision)
  */
static int32_t CalcVelocity(VelocityCalc_t *vc, uint32_t cnt)
{
    /* Measure actual time since last call using DWT cycle counter
     * SystemCoreClock = 84 MHz, so cycles_to_sec = cycles / 84000000 */
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t dt_cycles = now_cycles - vc->time_prev_cycles;
    vc->time_prev_cycles = now_cycles;

    /* Convert to seconds (float for precision) */
    float dt_sec = (float)dt_cycles / (float)SystemCoreClock;

    /* Sanity check: if dt is too small or too large, use nominal 1ms */
    if (dt_sec < 0.0001f || dt_sec > 0.1f) {
        dt_sec = 0.001f;
    }

    /* Calculate encoder delta */
    int32_t delta = (int32_t)cnt - (int32_t)vc->enc_prev;

    /* Handle wraparound based on encoder type */
    if (vc->wrap_at_cpr) {
        /* Motor encoder: wraps at CPR
         * If delta > cpr/2, counter wrapped backward
         * If delta < -cpr/2, counter wrapped forward */
        if (delta > (int32_t)(vc->cpr / 2)) {
            delta -= vc->cpr;
        } else if (delta < -(int32_t)(vc->cpr / 2)) {
            delta += vc->cpr;
        }
    }
    /* For 32-bit timer (TIM2), no wrap handling needed for normal operation */

    vc->enc_prev = cnt;

    /* Convert to counts/second using measured time delta */
    float vel_counts_per_sec = (float)delta / dt_sec;

    /* Apply exponential moving average filter */
    vc->vel_filtered = vc->filter_alpha * vel_counts_per_sec
                     + (1.0f - vc->filter_alpha) * vc->vel_filtered;

    /* Return in counts/second */
    return (int32_t)vc->vel_filtered;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
  * @brief  Initialize pendulum control subsystem
  */
void Pendulum_Init(void)
{
    /* Initialize cycle counter (DWT) */
    Chrono_Init();

    /* Clear SPI buffers */
    memset(spi_rx_buf, 0, SPI_BUFFER_SIZE);
    memset(spi_tx_buf, 0, SPI_BUFFER_SIZE);

    /* Reset state machine */
    g_state = STATE_START;
    spi_txrx_flag = 0;
    spi_err_flag = 0;

    /* Reset counters */
    err_cnt = 0;
    overtime_cnt = 0;
    cycle_cnt = 0;
    rotor_pos_cnt = 0;

    /* Initialize pendulum encoder */
    pendulum_enc.cnt = 0;
    pendulum_enc.previous_cnt = 0;
    pendulum_enc.position_steps = 0;
    pendulum_enc.position_init = 0;

    /* Initialize velocity calculator */
    pend_vel_calc.enc_prev = __HAL_TIM_GET_COUNTER(&htim2);
    pend_vel_calc.time_prev_cycles = DWT->CYCCNT;
    pend_vel_calc.vel_filtered = 0.0f;

#if DEBUG_PENDULUM_ENCODER || DEBUG_STEPPER
    Chrono_Mark(&debug_timer);
#endif

    /* Print startup message */
    UART_Print_Quick("\r\n=== Inverted Pendulum (Stepper) Started ===\r\n");

#if TEST_MODE_NO_MOTOR
    UART_Print_Quick("WARNING: TEST_MODE_NO_MOTOR is enabled - motor disabled\r\n");
#endif

#if SKIP_SPI_WAIT
    UART_Print_Quick("WARNING: SKIP_SPI_WAIT is enabled - running without SPI\r\n");
#endif

#if TEST_MODE_BUTTON
    UART_Print_Quick("Press blue button for acceleration test\r\n");
#endif
}

/**
  * @brief  Read pendulum encoder position
  */
void Pendulum_Encoder_Read(Pendulum_Encoder_TypeDef *enc, TIM_HandleTypeDef *htim)
{
    /* Save previous count */
    enc->previous_cnt = enc->cnt;

    /* Read current counter value (TIM2 is 32-bit) */
    enc->cnt = __HAL_TIM_GET_COUNTER(htim);

    /* Calculate delta (signed subtraction handles wrap correctly) */
    int32_t delta = (int32_t)(enc->cnt - enc->previous_cnt);

    /* Accumulate position (int32_t for extended range) */
    enc->position_steps += delta;

    /* Subtract zeroing offset */
    enc->position_steps -= enc->position_init;
}

/**
  * @brief  Start SPI DMA circular communication
  */
void SPI_StartCommunication(void)
{
    if (HAL_SPI_TransmitReceive_DMA(&hspi3, spi_tx_buf, spi_rx_buf, SPI_BUFFER_SIZE) != HAL_OK) {
        UART_Print_Quick("ERROR: SPI DMA init failed\r\n");
    }
}

/**
  * @brief  Run state machine iteration
  */
void StateMachine_Run(void)
{
#if !TEST_MODE_BUTTON
    /* Process any pending button presses (fault recovery) */
    ProcessButtonPress();
#endif

    switch (g_state)
    {
    /*------------------------------------------------------------------*/
    case STATE_START:
#if SKIP_SPI_WAIT
        /* Skip waiting for SPI - go directly to reading encoders */
        g_state = STATE_READ;
        Chrono_Mark(&cycle_timer);
        UART_Print_Quick("Skipping SPI wait, starting control loop...\r\n");
#else
        /* Wait for first SPI transaction to establish communication */
        if (spi_txrx_flag == 1) {
            spi_txrx_flag = 0;
            g_state = STATE_READ;
            Chrono_Mark(&cycle_timer);
            UART_Print_Quick("SPI connection established. Starting system...\r\n");
        }
#endif
        break;

    /*------------------------------------------------------------------*/
    case STATE_READ:
    {
        /* Read pendulum encoder */
        Pendulum_Encoder_Read(&pendulum_enc, &htim2);
        encoder_pos = pendulum_enc.position_steps;

        /* Calculate pendulum velocity using EMA filter */
        int32_t pend_vel_raw = CalcVelocity(&pend_vel_calc, pendulum_enc.cnt);
        int16_t pend_vel = (int16_t)(pend_vel_raw / PEND_VEL_RESOLUTION_DIV);

        /* Read rotor position (decimated to reduce SPI traffic to L6474) */
        if (rotor_pos_cnt % STEPPER_POSITION_DECIMATION == 0) {
            rotor_pos_cnt = 0;
            rotor_pos = GetPosition_L6474_Acceleration_Control();
        }
        rotor_pos_cnt++;

        /* Read rotor velocity (open-loop, directly from driver) */
        rotor_vel = GetVelocity_L6474_Acceleration_Control();

        /* Pack TX buffer (with IRQ protection for atomic update) */
        __disable_irq();
        send_pend_pos = swap16(saturate_int16(encoder_pos));
        send_pend_vel = swap16(pend_vel);
        send_rotor_pos = swap16(saturate_int16(rotor_pos));
        send_rotor_vel = swap16(rotor_vel);
        __enable_irq();

#if DEBUG_PENDULUM_ENCODER
        /* Periodically print encoder debug info */
        if (Chrono_GetDiffNoMark(&debug_timer) >= DEBUG_PRINT_INTERVAL_S) {
            Chrono_Mark(&debug_timer);
            int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                               "P:%ld pv:%d R:%ld rv:%d\r\n",
                               (long)encoder_pos, pend_vel,
                               (long)rotor_pos, rotor_vel);
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buf, (uint16_t)len, 1);
        }
#endif

#if DEBUG_STEPPER
        /* Periodically print stepper debug info */
        if (Chrono_GetDiffNoMark(&debug_timer) >= DEBUG_PRINT_INTERVAL_S) {
            Chrono_Mark(&debug_timer);
            int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                               "STEP: pos=%ld vel=%d accel=%d\r\n",
                               (long)rotor_pos, rotor_vel, accel_cmd);
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buf, (uint16_t)len, 1);
        }
#endif

#if SKIP_SPI_WAIT
        /* Without SPI, wait for sample interval then go to CONTROL */
        if (Chrono_GetDiffNoMark(&cycle_timer) >= T_SAMPLE) {
            Chrono_Mark(&cycle_timer);
            g_state = STATE_CONTROL;
        }
#else
        g_state = STATE_WAIT_SPI;
#endif
        break;
    }

    /*------------------------------------------------------------------*/
    case STATE_WAIT_SPI:
        __disable_irq();
        if (spi_txrx_flag == 1) {
            spi_txrx_flag = 0;
            accel_cmd = swap16(recv_accel_cmd);
            __enable_irq();
            g_state = STATE_CONTROL;
        } else {
            __enable_irq();
        }

        /* Check for overtime */
        if (Chrono_GetDiffNoMark(&cycle_timer) > OVERTIME_FACTOR * T_SAMPLE) {
            overtime_cnt++;
            g_state = STATE_OVERTIME;
#if DEBUG_STEPPER || DEBUG_PENDULUM_ENCODER
            int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                               "overtime: %" PRIu32 "\r\n", overtime_cnt);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_tx_buf, (uint16_t)len);
#endif
        }
        break;

    /*------------------------------------------------------------------*/
    case STATE_OVERTIME:
        /* Keep periodically reading rotor to check for overrange */
        if (Chrono_GetDiffNoMark(&cycle_timer) > ROTOR_CHECK_INTERVAL_S) {
            Chrono_Mark(&cycle_timer);
            rotor_pos = GetPosition_L6474_Acceleration_Control();
        }

        __disable_irq();
        if (spi_txrx_flag == 1) {
            spi_txrx_flag = 0;
            accel_cmd = swap16(recv_accel_cmd);
            __enable_irq();
            Chrono_Mark(&cycle_timer);
            g_state = STATE_CONTROL;
        } else {
            __enable_irq();
        }
        break;

    /*------------------------------------------------------------------*/
    case STATE_CONTROL:
    {
#if TEST_MODE_BUTTON
        /* Button test override */
        if (button_test_active) {
            if ((HAL_GetTick() - button_test_start_tick) < BUTTON_TEST_DURATION_MS) {
                accel_cmd = BUTTON_TEST_ACCEL_VALUE;
            } else {
                accel_cmd = 0;
                button_test_active = 0;
            }
        }
#else
        /* If rotor fault active, force zero acceleration */
        if (rotor_fault_active) {
            accel_cmd = 0;
        }
#endif

#if TEST_MODE_NO_MOTOR
        /* Motor disabled - don't send any commands */
        (void)accel_cmd;
#else
        /* Apply acceleration command to stepper motor */
#ifdef ACCELERATION_CONTROL
        Run_L6474_Acceleration_Control(accel_cmd);
#endif

#ifdef POSITION_CONTROL
        /* Run position control if motor inactive or standby */
        if (BSP_MotorControl_GetDeviceState(0) >= 8) {
            BSP_MotorControl_GoTo(0, (int32_t)accel_cmd);
        }
#endif
#endif /* TEST_MODE_NO_MOTOR */

        /* Restart cycle */
        Chrono_Mark(&cycle_timer);
        cycle_cnt++;
        g_state = STATE_READ;
        break;
    }

    /*------------------------------------------------------------------*/
    case STATE_ERROR_ROTOR:
    case STATE_HALT:
        /* These states are no longer used - fault is handled via rotor_fault_active flag */
        /* If we somehow end up here, go back to normal operation */
        g_state = STATE_READ;
        break;

    /*------------------------------------------------------------------*/
    default:
        /* Unknown state - reset to start */
        g_state = STATE_START;
        break;
    }

    /* Post-process step clock (must be called every loop) */
#if !TEST_MODE_NO_MOTOR
    PostProcess_StepClockHandler_L6474_Acceleration_Control();
#endif

    /* Safety check - set fault flag if rotor exceeds safe range */
#if !TEST_MODE_BUTTON
    if (!rotor_fault_active) {
        if ((float)labs(rotor_pos) > MAX_DEFLECTION_REV * STEPS_PER_TURN) {
            rotor_fault_active = 1;
            fault_recovery_phase = 0;
            BSP_MotorControl_HardStop(0);  /* Stop motor but keep powered (holds position) */
            UART_Print_Quick("FAULT: Rotor out of range. Press button to reset\r\n");
        }
    }
#else
    /* When TEST_MODE_BUTTON is active, just transition to error state */
    if ((float)labs(rotor_pos) > MAX_DEFLECTION_REV * STEPS_PER_TURN) {
        if (g_state != STATE_HALT && g_state != STATE_ERROR_ROTOR) {
            BSP_MotorControl_HardStop(0);
            UART_Print_Quick("FAULT: Rotor out of range\r\n");
            g_state = STATE_HALT;
        }
    }
#endif

    /* Count SPI errors */
    if (spi_err_flag) {
        err_cnt++;
        spi_err_flag = 0;
    }
}

/**
  * @brief  Get current acceleration command
  */
int16_t GetAccelCommand(void)
{
    return accel_cmd;
}

/**
  * @brief  Get current pendulum position
  */
int32_t GetPendulumPosition(void)
{
    return pendulum_enc.position_steps;
}

/**
  * @brief  Get current rotor position
  */
int32_t GetRotorPosition(void)
{
    return rotor_pos;
}

/*******************************************************************************
 * HAL Callbacks
 ******************************************************************************/

/**
  * @brief  SPI transmit/receive complete callback
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {
        spi_txrx_flag = 1;

        /* Copy send values to TX buffer (4 x int16 = 8 bytes)
         * Order: pend_pos, pend_vel, rotor_pos, rotor_vel */
        memcpy(&spi_tx_buf[0], &send_pend_pos, 2);
        memcpy(&spi_tx_buf[2], &send_pend_vel, 2);
        memcpy(&spi_tx_buf[4], &send_rotor_pos, 2);
        memcpy(&spi_tx_buf[6], &send_rotor_vel, 2);

        /* Copy RX buffer to recv values
         * Only first int16 is accel_cmd, rest is reserved */
        memcpy(&recv_accel_cmd, &spi_rx_buf[0], 2);
    }
}

/**
  * @brief  SPI error callback
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {
        spi_err_flag = 1;
    }
}

/**
  * @brief  UART transmit complete callback
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        uart_xmit_flag = 1;
    }
}

/**
  * @brief  UART error callback
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        uart_err_flag = 1;
    }
}

#if TEST_MODE_BUTTON
/**
  * @brief  Button press handler for test mode
  * @note   Call this from EXTI callback for user button (PC13)
  */
void Button_TestMode_Handler(void)
{
    static uint32_t last_press_tick = 0;
    uint32_t now = HAL_GetTick();

    /* Debounce: ignore presses within 200ms */
    if ((now - last_press_tick) < 200) {
        return;
    }
    last_press_tick = now;

    if (!button_test_active) {
        button_test_active = 1;
        button_test_start_tick = now;
    }
}
#else
/**
  * @brief  Button press handler for fault recovery (ISR context)
  * @note   Call this from EXTI callback for user button (PC13)
  *         Only sets a flag - actual processing done in main loop to avoid
  *         calling BSP_MotorControl functions (which use SPI) from ISR
  */
void Button_HaltRecovery_Handler(void)
{
    static uint32_t last_press_tick = 0;
    uint32_t now = HAL_GetTick();

    /* Debounce: ignore presses within 200ms */
    if ((now - last_press_tick) < 200) {
        return;
    }
    last_press_tick = now;

    /* Only set flag if we're in fault state */
    if (rotor_fault_active) {
        button_press_pending = 1;
    }
}

/**
  * @brief  Process pending button press (called from main loop)
  * @note   BSP_MotorControl functions use SPI and must not be called from ISR
  */
static void ProcessButtonPress(void)
{
    if (!button_press_pending) {
        return;
    }
    button_press_pending = 0;

    if (rotor_fault_active && fault_recovery_phase == 0) {
        /* First button press: disable motor power only
         * User can now manually move rotor to center position */
        BSP_MotorControl_CmdDisable(0);      /* HiZ mode - motor can be moved manually */
        fault_recovery_phase = 1;
        UART_Print_Quick("Motor disabled. Move rotor to center, then press button\r\n");
    }
    else if (rotor_fault_active && fault_recovery_phase == 1) {
        /* Second button press: reset position (now at center), enable motor, clear fault */
        BSP_MotorControl_CmdSetParam(0, L6474_ABS_POS, 0);  /* Write 0 to ABS_POS register */
        Reset_L6474_Acceleration_Control();  /* Reset accel control state to 0 */
        rotor_pos = 0;                       /* Reset local position variable */
        BSP_MotorControl_CmdEnable(0);       /* Re-enable motor driver */
        rotor_fault_active = 0;              /* Clear fault flag */
        fault_recovery_phase = 0;
        UART_Print_Quick("Position reset, motor enabled. Waiting for first non-zero command\r\n");
    }
}
#endif
