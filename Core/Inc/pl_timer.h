/*
 * pl_timer.h
 *
 *  Created on: Dec 10, 2022
 *      Author: kawaguchitakahito
 */

#ifndef INC_PL_TIMER_H_
#define INC_PL_TIMER_H_

#include "stm32l4xx_hal.h"

extern volatile uint32_t g_timCount;


void pl_timer_init();

void pl_timer_count();
void wait_ms(uint32_t);


#endif /* INC_PL_TIMER_H_ */
