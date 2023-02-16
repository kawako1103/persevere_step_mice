/*
 * speaker.c
 *
 *  Created on: 2023/02/13
 *      Author: kawaguchitakahito
 */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


void test_speaker(void){

	 if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_Delay(100);
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
		while(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==1){
		}
		HAL_Delay(100);
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	 }
}

void ring_step(void){
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}

void ring_start(void){
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(1500);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}

void ring_caution(void){
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
//	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
//	HAL_Delay(500);
//	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
//	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
//	HAL_Delay(500);
//	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
//	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
//	HAL_Delay(500);
//	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);

}

void ring_end(void){
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}

void ring_interrupt(void){
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}
