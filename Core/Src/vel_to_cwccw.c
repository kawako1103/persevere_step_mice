/*
 * vel_to_cwccw.c
 *
 *  Created on: Dec 15, 2022
 *      Author: kawaguchitakahito
 */

#include "vel_to_cwccw.h"
#include "gpio.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

uint16_t leftCWCCW(float velocity) {//左のタイヤが正転or逆転
	  if ((velocity) > 0.0)
	  {
		  HAL_GPIO_WritePin(MOTOR_CWORCCW_L_GPIO_Port,MOTOR_CWORCCW_L_Pin,GPIO_PIN_RESET);//これ逆かも
		  //HAL_Delay(1000);
	  } else if((velocity) < 0.0)
	  {
		  HAL_GPIO_WritePin(MOTOR_CWORCCW_L_GPIO_Port,MOTOR_CWORCCW_L_Pin,GPIO_PIN_SET);
		  //HAL_Delay(1000);
	  }
}

uint16_t rightCWCCW(float velocity) {//右のタイヤが正転or逆転
	  if ((velocity) > 0.0)
	  {
		  HAL_GPIO_WritePin(MOTOR_CWORCCW_R_GPIO_Port,MOTOR_CWORCCW_R_Pin,GPIO_PIN_SET);
		  //HAL_Delay(1000);
	  } else if((velocity) < 0.0)
	  {
		  HAL_GPIO_WritePin(MOTOR_CWORCCW_R_GPIO_Port,MOTOR_CWORCCW_R_Pin,GPIO_PIN_RESET);
		  //HAL_Delay(1000);
	  }
}



