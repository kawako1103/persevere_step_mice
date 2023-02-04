/*
 * pl_timer.c
 *
 *  Created on: Dec 10, 2022
 *      Author: kawaguchitakahito
 */
#include "tim.h"

#include "pl_timer.h"
volatile uint32_t g_timCount;

void pl_timer_init(void){
	HAL_TIM_Base_Start_IT(&htim6);
}
void pl_timer_count(void){//カウント
	g_timCount++;
}
void wait_ms(uint32_t wait_time){//割り込みの
	g_timCount = 0;
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (g_timCount < wait_time){
		HAL_TIM_Base_Start_IT(&htim15);//speaker
		HAL_TIM_PWM_MspInit(&htim15);//speaker
	}
}

