/*
 * vel_and_dis_integral.c
 *
 *  Created on: Dec 14, 2022
 *      Author: kawaguchitakahito
 */
#include "adc.h"
#include "dma.h"
#include "gpio.h"//
#include "vel_and_dis_integral.h"

float dis_integral;
float vel_integral;
float acc_integral;

//構造体tarparameterをやめた　一つ一つ返すことにした

const float DT = 0.001;

//void calPara(tarparameter *para) {
//	dis_integral += vel_integral * DT +
//			acc_integral * DT * DT / 2.0;
//	vel_integral += acc_integral * DT;
//}

void calPara(dis_integral,vel_integral,acc_integral) {
	dis_integral += vel_integral * DT +
			acc_integral * DT * DT / 2.0;
	vel_integral += acc_integral * DT;
}

