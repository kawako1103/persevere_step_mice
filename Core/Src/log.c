/*
 * log.c
 *
 *  Created on: Dec 16, 2022
 *      Author: kawaguchitakahito
 */
#include "trapezoid_acc_model.h"

int log_vel_flg;//flgの宣言
float record[5000];
int t=0;//記録用配列
void log_vel_interupt(void){
	if (log_vel_flg==1){
		  record[t] = vel;
		  t += 1;
	  }else{
	  }
	return 0;
}
