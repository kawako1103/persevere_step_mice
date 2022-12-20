/*
 * Motor_Run.c
 *
 *  Created on: 2022/12/19
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
#include "math.h"


//int target_dis=500;//設定距離

//任意距離前進調整はタイヤ系で 壁制御あり
void trapezoid_accel_forward(float a,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
//	   acc=1000;//加速度の定義
//	   v_start=100;//初速定義
//	   v_max=500;//最高速度定義
//	   v_end=100;//終端速度定義
//	   x=540;//目標

	   //初期化
	   acc=a;
	   v_start=v0;
	   v_max=vM;
	   v_end=vE;
	   target_dis=tx;
	   float x_dec=0;
	   dt=0.001;//刻み時間
	   dis=0;
	   vel =v_start;
	   //壁制御オン(1)
	   wall_control_flg=1;
	    HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
		HAL_Delay(3);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		trapezoid_flg=1;
		x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
		while(vel < v_max){
		}
		acc=0;
		while(target_dis-dis>x_dec){
			printf("%f\n\r",vel);
		}
		acc=-1000;
		while(vel>v_end){
			printf("%f\n\r",vel);
		}
		acc=0;
		trapezoid_flg=0;
	//				HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(500);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		//壁制御用のflgをオフ(0)にする
		wall_control_flg=0;
	 }

void trapezoid_accel_backward(float a,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
//	   acc=1000;//加速度の定義
//	   v_start=100;//初速定義
//	   v_max=500;//最高速度定義
//	   v_end=100;//終端速度定義
//	   x=540;//目標

	   //初期化
	   acc=-a;
	   v_start=-v0;
	   v_max=-vM;
	   v_end=-vE;
	   target_dis=-tx;
	   float x_dec=0;
	   dt=0.001;//刻み時間
	   dis=0;
	   vel =-v_start;
	   //壁制御オン(1)
	   	wall_control_flg=1;
	    HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
		HAL_Delay(3);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		trapezoid_flg=1;
		x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
		while(fabs(vel) < fabs(v_max)){
		}
		acc=0;
		while(fabs(target_dis)-fabs(dis)>fabs(x_dec)){
			printf("%f\n\r",vel);
		}
		acc=-1000;
		while(fabs(vel)>fabs(v_end)){
			printf("%f\n\r",vel);
		}
		acc=0;
		trapezoid_flg=0;
	//				HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(500);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		//壁制御用のflgをオフ(0)にする
	    wall_control_flg=0;
	 }


void trapezoid_accel_lturn(float angle_a,float angle_v0,float angle_vM,float angle_vE,float angle_t){//超信地旋回左から角加速度,初角速,最大角速度,終端角速度,設定角度
	   //一応初期化
	   acc=0;//加速度の定義
	   v_start=0;//初速定義
	   v_max=0;//最高速度定義
	   v_end=0;//終端速度定義
	   target_dis=0;//目標
	   dis=0;
	   vel =0;
//	   //以下角速度系初期化
//	   angle_acc=1000;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
//	   angle_v_start=100;//初角速度定義
//	   angle_v_max=400;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
//	   angle_v_end=80;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
//	   target_angle=180;//目標 180°
//	   dt=0.001;//刻み時間
//	   angle=0;//変数としての角度
//	   angle_vel =10;//変数としての角速度

	   angle_acc=angle_a;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
	   angle_v_start=angle_v0;//初角速度定義
	   angle_v_max=angle_vM;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
	   angle_v_end=angle_vE;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
	   target_angle=angle_t;//目標 180°
	   dt=0.001;//刻み時間
	   angle=0;//変数としての角度
	   angle_vel =angle_v_start;//変数としての角速度

	   //wall_control_flg=1;//壁制御 多分ここでは不要　超信地旋回の時は一旦壁制御切っても良いのでは

		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
		HAL_Delay(3);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		trapezoid_angle_flg=1;
		angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);
		while(angle_vel < angle_v_max){
			//printf("%f\n\r",angle);
		}
		angle_acc=0;
		while(target_angle-angle > angle_dec){
			//printf("%f\n\r",angle);
		}
		angle_acc=-2000;
		while(angle_vel>angle_v_end){
			//printf("%f\n\r",angle);
		}
		angle_acc=0;
		trapezoid_angle_flg=0;
//				HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(500);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		wait_ms(500);
		//壁制御用のflgをオフ(0)にする
		//wall_control_flg=0;
}

void trapezoid_accel_rturn(float angle_a,float angle_v0,float angle_vM,float angle_vE,float angle_t){//超信地旋回左から角加速度,初角速,最大角速度,終端角速度,設定角度
	   //一応初期化
	   acc=0;//加速度の定義
	   v_start=0;//初速定義
	   v_max=0;//最高速度定義
	   v_end=0;//終端速度定義
	   target_dis=0;//目標
	   dis=0;
	   vel =0;
	   //以下角速度系初期化
//	   angle_acc=1000;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
//	   angle_v_start=100;//初角速度定義
//	   angle_v_max=400;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
//	   angle_v_end=80;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
//	   target_angle=180;//目標 180°
//	   dt=0.001;//刻み時間
//	   angle=0;//変数としての角度
//	   angle_vel =10;//変数としての角速度

	   angle_acc=-angle_a;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
	   angle_v_start=-angle_v0;//初角速度定義
	   angle_v_max=-angle_vM;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
	   angle_v_end=-angle_vE;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
	   target_angle=-angle_t;//目標回転角度
	   dt=0.001;//刻み時間
	   angle=0;//変数としての角度
	   angle_vel =angle_v_start;//変数としての角速度

	   //wall_control_flg=1;//壁制御 多分ここでは不要　超信地旋回の時は一旦壁制御切っても良いのでは

		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
		HAL_Delay(3);
		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		trapezoid_angle_flg=1;
		angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);
		while(fabs(angle_vel) < fabs(angle_v_max)){
			//printf("%f\n\r",angle);
		}
		angle_acc=0;
		while(fabs(target_angle)-fabs(angle) > fabs(angle_dec)){
			//printf("%f\n\r",angle);
		}
		angle_acc=angle_a;
		while(fabs(angle_vel)>fabs(angle_v_end)){
			//printf("%f\n\r",angle);
		}
		angle_acc=0;
		trapezoid_angle_flg=0;
//				HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(500);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		wait_ms(500);
		//壁制御用のflgをオフ(0)にする
		//wall_control_flg=0;
}

