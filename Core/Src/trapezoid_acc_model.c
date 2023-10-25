/*
 * trapezoid_acc_model.c
 *
 *  Created on: Dec 14, 2022
 *      Author: kawaguchitakahito
 */
#include "trapezoid_acc_model.h"
#include "calPWMvel.h"
#include "wall_control.h"
#include "vel_to_cwccw.h"

float acc=1000;//加速度の定義
float v_start=100;//初速定義
float v_max=500;//最高速度定義
float v_end=300;//終端速度定義
float x=500;//設定距離
float dt=0.001;//刻み時間
float dis;//disに代入する形なので特に値を代入する必要ない
float vel =100;
float angle;//disに代入する形なので特に値を代入する必要ない
float angular_vel=30;
float angular_acc=100;
float x_acc;//加速距離
float x_dec;//減速距離
float Delta_x;
int trapezoid_flg;//台形加速用のflgの宣言
int trapezoid_angle_flg;//超信地旋回(台形角加速)用のflgの宣言
float g_motorCount_l=12200;
float g_motorCount_r=12200;
float tread_width = 8.15;

//void acc_dis(a,v_start,v_max){
//x_acc = (v_max^2-v_start^2)/(2*a);
//}
//
//void dec_dis(a,v_end,v_max){
//x_dec = (v_max^2-v_end^2)/(2*a);
//}


void trapezoid_acc_interupt(){
	if (trapezoid_flg==1){
//		  PID_wall = calWallControl();
		  dis += vel * dt;
		  vel += acc * dt;
		  leftCWCCW(vel);
		  rightCWCCW(vel);
		  g_motorCount_l = (float)calPWMCount(vel+PID_wall);
		  g_motorCount_r = (float)calPWMCount(vel-PID_wall);

	  }else{
	  }
}

void trapezoid_angle_acc_interupt(){
	if (trapezoid_angle_flg==1){
//		  PID_wall = calWallControl();
		  vel=0;
		  angle += angular_vel * dt;
		  angular_vel += angular_acc * dt;
		  leftCWCCW(angular_vel);
		  rightCWCCW(angular_vel);
		  g_motorCount_l = (float)calPWMCount(vel+PID_wall-angular_vel*tread_width/2);
		  g_motorCount_r = (float)calPWMCount(vel-PID_wall+angular_vel*tread_width/2);

	  }else{
	  }
}
