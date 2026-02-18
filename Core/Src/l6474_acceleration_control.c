#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;
static Chrono_TypeDef cycletimer= {0,0,0.0};
static volatile bool ISRFlag = FALSE;

static volatile uint8_t lastTimHALstate = 0;

extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId); // standard stepclockhandler
extern uint16_t L6474_StepClockHandler_alt(uint8_t deviceId, int16_t acceleration); // alternative stepclockhandler for acceleration control
extern void L6474_ApplySpeed(uint8_t pwmId, uint16_t newSpeed);

void Init_L6472_Acceleration_Control(L6474_Init_t *gInitParams, float t_sample) {

	/// Acceleration rate in step/s2. Range: (0..+inf).
	hAccelCtrl.max_accel = gInitParams->acceleration_step_s2;
	/// DECELERATION is ignored for acceleration control

	/// Maximum speed in step/s. Range: (30..10000].
	hAccelCtrl.max_speed = gInitParams->maximum_speed_step_s;
	 ///Minimum speed in step/s. Range: [30..10000).
	hAccelCtrl.min_speed = gInitParams->minimum_speed_step_s;


	  hAccelCtrl.speed = hAccelCtrl.min_speed;
	  hAccelCtrl.period = 84000000 / ( 1024 * (uint32_t)hAccelCtrl.min_speed);

	  hAccelCtrl.t_sample = t_sample;
	  cycletimer.t_diff_s = hAccelCtrl.t_sample;

	  hAccelCtrl.state = 0;

	  hAccelCtrl.acceleration = 0;
	  hAccelCtrl.velocity = 0;
	  hAccelCtrl.position = 0;
}

void Stop_L6472_Acceleration_Control(void){
	BSP_MotorControl_HardStop(0);
	hAccelCtrl.state = 0;
}

void Run_L6472_Acceleration_Control(int16_t acceleration_input) {

	switch (hAccelCtrl.state) {
	case 0:
		if (acceleration_input !=0) {
			hAccelCtrl.state = 1;
		}
		break;
	case 1: // first call
		hAccelCtrl.state = 10;
		BSP_MotorControl_Run(0,FORWARD);
		ISRFlag = FALSE;
		Chrono_Mark(&cycletimer);
		break;
	case 10: // normal operation
		hAccelCtrl.t_sample = Chrono_GetDiffMark(&cycletimer);

	  // clamp acceleration
	  if (acceleration_input > (int16_t) hAccelCtrl.max_accel) {
		  hAccelCtrl.acceleration = (int16_t) hAccelCtrl.max_accel;
	  } else if (acceleration_input < -1.0*(int16_t) hAccelCtrl.max_accel) {
		  hAccelCtrl.acceleration = -1.0* (int16_t) hAccelCtrl.max_accel;
	  } else {
		  hAccelCtrl.acceleration = acceleration_input;
	  }

		break;
	default:
		break;
	}
}

// this function is called from timer ISR
void StepClockHandler_L6472_Acceleration_Control(void) {

	ISRFlag = TRUE;

	hAccelCtrl.speed = L6474_StepClockHandler_alt(0, hAccelCtrl.acceleration);
	hAccelCtrl.period = 84000000 / ( 1024 * (uint32_t)hAccelCtrl.speed);
	L6474_Board_Pwm1StartIT();

}

// this function is called every main cycle to check for step updates
void PostProcess_StepClockHandler_L6472_Acceleration_Control(void) {

	if (ISRFlag && hAccelCtrl.state == 10 && L6474_Board_Pwm1GetCounter() <= 5) {
		// only update autoreload when counter is near 0
		ISRFlag = FALSE;
		L6474_Board_Pwm1SetAutoReload(hAccelCtrl.period);
		L6474_Board_Pwm1SetCompare(hAccelCtrl.period);

		if (BSP_MotorControl_GetDirection(0) == FORWARD) {
			hAccelCtrl.velocity = (int32_t) hAccelCtrl.speed;
		} else {
			hAccelCtrl.velocity = (int32_t) -1 * hAccelCtrl.speed;
		}
	}

}

float GetSampleTime_L6472_Acceleration_Control(void) {
	return hAccelCtrl.t_sample;
}

int32_t GetPosition_L6472_Acceleration_Control(void){
 // warning, involves SPI, do not call every cycle
	hAccelCtrl.position = BSP_MotorControl_GetPosition(0);
	return hAccelCtrl.position;
}

int16_t GetVelocity_L6472_Acceleration_Control(void){
	return hAccelCtrl.velocity;
}

int16_t GetAcceleration_L6472_Acceleration_Control(void){
	return hAccelCtrl.acceleration;
}
