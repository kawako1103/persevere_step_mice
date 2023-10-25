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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Motor_Run.h"
#define PI 3.1415926535
//台形加速用パラメータ
float acc=2000;//加速度の定義
float a0;
float v_start=100;//初速定義
float v_max=500;//最高速度定義
float v_end=300;//終端速度定義
float v_dec=0;//初期化的な宣言
float x_dec;
float target_dis;//設定距離
float left_dis;
float dt=0.001;//刻み時間
float dis;//disに代入する形なので特に値を代入する必要ない
float vel =100;
//超信地旋回用パラメータ
float angle_v_start=100;//初角速度定義
float angle_v_max=540;//最高角速度定義
float angle_v_end=200;//終端角速度定義
float target_angle=180;//設定角度
float angle;//disに代入する形なので特に値を代入する必要ない
float angle_vel=30;//一旦
float angle_acc=1000;//一旦
float angle_acc;//加速角度
float angle_dec;//減速角度
float rw;//回転速度的な
//float Delta_x;//イラン
int trapezoid_flg;//台形加速用のflgの宣言
int trapezoid_angle_flg;//超信地旋回(台形角加速)用のflgの宣言
int slalom_trapezoid_flg;//スラローム用のflgの宣言
float g_motorCount_l=12200.0;
float g_motorCount_r=12200.0;
float tread_width = 84.5*330/360*375/360*355/360;//トレッドミルは8.15cm=81.5 120°/60.5→95°.66°/90°が120°/90.5 150°/100.5 175°くらい/103.5 181°くらい/90°で調整すべきか 今のままだと90°が150°くらい//防音室では、調整して81.5→82.5→84.5大きくしたちょっと

//void acc_dis(a,v_start,v_max){
//x_acc = (v_max^2-v_start^2)/(2*a);
//}
//
//void dec_dis(a,v_end,v_max){
//x_dec = (v_max^2-v_end^2)/(2*a);
//}


//台形加速割り込み用
void trapezoid_acc_interupt(){
	if (trapezoid_flg==1){
//		  PID_wall = calWallControl();

	      x_dec = (vel*vel-v_end*v_end)/(2*accm);
		  left_dis=target_dis-dis;
		  dis += vel * dt;
		  vel += acc * dt;
		  leftCWCCW(vel);
		  rightCWCCW(vel);
		  g_motorCount_l = (float)calPWMCount(vel+PID_wall);
		  g_motorCount_r = (float)calPWMCount(vel-PID_wall);

	  }else{
	  }
}


//void trapezoid_accel(int a,int v0,int vM,int vE,int tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
////	   acc=1000;//加速度の定義
////	   v_start=100;//初速定義
////	   v_max=500;//最高速度定義
////	   v_end=100;//終端速度定義
////	   x=540;//目標
//
//	   //初期化
//	   acc=a;
//	   v_start=v0;
//	   v_max=vM;
//	   v_end=vE;
//	   target_dis=tx;
//
//	   dt=0.001;//刻み時間
////	   dis=0;
//	   vel =100;
//
//		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//		trapezoid_flg=1;
//		float x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
//		while(vel < v_max){
//		}
//		acc=0;
//		while(target_dis-dis>x_dec){
//			//printf("%f\n\r",vel);
//		}
//		acc=-1000;
//		while(vel>v_end){
//			//printf("%f\n\r",vel);
//		}
//		acc=0;
//		trapezoid_flg=0;
////				HAL_Delay(1000);
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//
//}


//超信地旋回割り込み用(台形角加速)
void trapezoid_angle_acc_interupt(){
	if (trapezoid_angle_flg==1){
//		  PID_wall = calWallControl();//余談だけどやっていたミスとしてここで初期化すると割り込みのたびに角度0で振り出しに戻るので、止まらなくなる
		  angle += angle_vel * dt;
		  angle_vel += angle_acc * dt;
		  rw=angle_vel*PI/180*tread_width/2;
		  leftCWCCW(-rw);
		  rightCWCCW(rw);
		  g_motorCount_l = (float)calPWMCount(vel+PID_wall-rw);
		  g_motorCount_r = (float)calPWMCount(vel-PID_wall+rw);
	  }else{
	  }
}


//スラローム
void slalom_trapezoid_acc_interupt(){//台形加速割り込み用
	if (slalom_trapezoid_flg==1){
//		  PID_wall = calWallControl();


			angle += angle_vel * dt;
			angle_vel += angle_acc * dt;
			rw=angle_vel*PI/180*tread_width/2;

	      x_dec = (vel*vel-v_end*v_end)/(2*accm);
		  left_dis=target_dis-dis;
		  dis += vel * dt;
		  vel += acc * dt;
		  leftCWCCW(vel-rw);
	      rightCWCCW(vel+rw);
		  g_motorCount_l = (float)calPWMCount(vel+PID_wall-rw);
		  g_motorCount_r = (float)calPWMCount(vel-PID_wall+rw);

	  }else{
	  }
}
