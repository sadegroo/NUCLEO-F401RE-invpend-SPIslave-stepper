

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L6474_ACCELERATION_CONTROL_H
#define __L6474_ACCELERATION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// includes
#include <math.h>
#include "motor.h"
#include "l6474.h"
#include "x_nucleo_ihm01a1_stm32f4xx.h"
#include "x_nucleo_ihmxx.h"
#include <chrono.h>

// defines
#define ARM_MATH_CM4
#define ACCEL_CONTROL_DATA 0		// Set to 1 for display of timing data
#define PWM_COUNT_SAFETY_MARGIN 2

// Prescale a frequency in preparation for calculating counter period for PWM1
#define __L6474_Board_Pwm1PrescaleFreq(freq) (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER * freq)

//typedefs
typedef struct {
	volatile int16_t acceleration; // in microsteps/s^2, can be positive and negative
	volatile int16_t velocity;	// in microsteps/s, can be positive and negative
	volatile int32_t position;	// in microsteps, can be positive and negative
	volatile uint16_t speed; // in microsteps/s, always positive
	volatile uint32_t period; // in timer increments, always positive
	volatile uint16_t min_speed; // in microsteps/s
	volatile uint16_t max_speed; // in microsteps/s
	volatile uint16_t max_accel; // in microsteps/s^2
	volatile float t_sample; // in seconds
	volatile uint8_t state;

} L6474_Acceleration_Control_TypeDef;

// local function prototypes
void Init_L6472_Acceleration_Control(L6474_Init_t *gInitParams, float t_sample);
void Run_L6472_Acceleration_Control(int16_t acceleration_input);
void StepClockHandler_L6472_Acceleration_Control(void);
void PostProcess_StepClockHandler_L6472_Acceleration_Control(void);
float GetSampleTime_L6472_Acceleration_Control(void);
int32_t GetPosition_L6472_Acceleration_Control(void);
int16_t GetVelocity_L6472_Acceleration_Control(void);
int16_t GetAcceleration_L6472_Acceleration_Control(void);

// extern function prototypes


#ifdef __cplusplus
}
#endif

#endif /* __L6474_ACCELERATION_CONTROL_H */
