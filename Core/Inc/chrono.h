

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHRONO_H
#define __CHRONO_H

#ifdef __cplusplus
extern "C" {
#endif

// includes
#include <stdint.h>
#include "stm32f4xx_nucleo.h"

// defines
#define RCC_SYS_CLOCK_FREQ 84000000 // should equal HAL_RCC_GetSysClockFreq()
#define RCC_HCLK_FREQ 84000000 // should equal HAL_RCC_GetHCLKFreq()

//typedefs
typedef struct {
	uint32_t start_ticks;
	uint32_t end_ticks;
	float t_diff_s;
}Chrono_TypeDef;

// local function prototypes
void Chrono_Init(void);
void Chrono_Mark(Chrono_TypeDef * chrono);
float Chrono_GetDiffNoMark(Chrono_TypeDef * chrono);
float Chrono_GetDiffMark(Chrono_TypeDef * chrono);

// extern function prototypes

#ifdef __cplusplus
}
#endif

#endif /* __CHRONO_H */
