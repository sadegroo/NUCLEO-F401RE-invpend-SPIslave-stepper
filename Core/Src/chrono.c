
#include <chrono.h>

void Chrono_Init(void) {
  // Initialize and enable cycle counter
  ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
  DWT->CTRL |= 1; 		// at address 0xE0001000
  DWT->CYCCNT = 0; 		// at address 0xE0001004
}

void Chrono_Mark(Chrono_TypeDef * chrono) {
	//measure time difference
	chrono->end_ticks = DWT->CYCCNT; // Read the current CYCCNT value
	uint32_t difference = chrono->end_ticks - chrono->start_ticks;

	// Update the previous value for the next measurement
	chrono->start_ticks = chrono->end_ticks;

	// Convert difference to time (in seconds)
	chrono->t_diff_s = (float)difference / RCC_SYS_CLOCK_FREQ;
}

float Chrono_GetDiffNoMark(Chrono_TypeDef * chrono) {
	//measure time difference
	uint32_t current_ticks =DWT->CYCCNT; // Read the current CYCCNT value
	uint32_t difference = current_ticks - chrono->start_ticks;

	// Convert difference to time (in seconds)
	return (float)difference / RCC_SYS_CLOCK_FREQ;
}

float Chrono_GetDiffMark(Chrono_TypeDef * chrono) {
	Chrono_Mark(chrono);
	return chrono->t_diff_s;
}

