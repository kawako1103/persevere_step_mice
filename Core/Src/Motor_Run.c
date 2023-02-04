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
#include "explore_method.h"

float accm;

//int target_dis=500;//設定距離

//励磁、PWM系のONOFF
void motor_excitation_on(){//励磁ON タイヤを固める
	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
}

void motor_excitation_off(){//励磁ON タイヤを緩める(これは動いている時にやると慣性で吹っ飛ぶので要注意)
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
}


void motor_pwm_on(){//PWM系のON モータの励磁をONにしたりOFFにしたりを繰り返す　つまり、車輪を回転させる
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void motor_pwm_off(){//PWM系のON モータの励磁をONにしたりOFFにしたりを繰り返すのをやめる　つまり、車輪の回転を停止させる
	//				HAL_Delay(1000);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_Delay(300);//300→10
//		HAL_Delay(500);
}


//連続足立法にあたり、修正後段階の台形加速(励磁,PWMonoffが入ってる)
//任意距離前進調整はタイヤ系で 壁制御あり
void trapezoid_accel_forward(float a0,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
//	   acc=1000;//加速度の定義
//	   v_start=100;//初速定義
//	   v_max=500;//最高速度定義
//	   v_end=100;//終端速度定義
//	   x=540;//目標

	   //初期化
	   accm=a0;//externする用
	   acc=a0;
	   v_start=v0;
	   v_max=vM;
	   v_end=vE;
	   target_dis=tx;
	   dt=0.001;//刻み時間
	   dis=0;
	   left_dis=target_dis;
	   vel =v_start;
	   //壁制御オン(1)
	   wall_control_flg=1;


		trapezoid_flg=1;
//		motor_pwm_on();

//		x_dec = (vel*vel-v_end*v_end)/(2*a);
//		printf("%f\n\r",vel);
		while((vel < v_max)&&(left_dis>x_dec)){
//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);

		}
		if(vel > v_max){
			vel=vM;
		}
		acc=0;
		while(left_dis>x_dec){//
			//printf("%f\n\r",vel);
//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
		}
		acc=-a0;
		while(vel>v_end){
			//printf("%f\n\r",vel);
//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
		}
//		acc=0;
		trapezoid_flg=0;
//		motor_pwm_off();//これがあると止まってしまう

		//壁制御用のflgをオフ(0)にする
		wall_control_flg=0;
	 }


////初期段階の台形加速(励磁,PWMonoffが入ってる)
////任意距離前進調整はタイヤ系で 壁制御あり
//void trapezoid_accel_forward(float a0,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
////	   acc=1000;//加速度の定義
////	   v_start=100;//初速定義
////	   v_max=500;//最高速度定義
////	   v_end=100;//終端速度定義
////	   x=540;//目標
//
//	   //初期化
//	   accm=a0;//externする用
//	   acc=a0;
//	   v_start=v0;
//	   v_max=vM;
//	   v_end=vE;
//	   target_dis=tx;
//	   dt=0.001;//刻み時間
//	   dis=0;
//	   left_dis=target_dis;
//	   vel =v_start;
//	   //壁制御オン(1)
//	   wall_control_flg=1;
//
//	    HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		trapezoid_flg=1;
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
////		x_dec = (vel*vel-v_end*v_end)/(2*a);
////		printf("%f\n\r",vel);
//		while((vel < v_max)&&(left_dis>x_dec)){
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//
//		}
//		if(vel > v_max){
//			vel=vM;
//		}
//		acc=0;
//		while(left_dis>x_dec){//
//			//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//		}
//		acc=-a0;
//		while(vel>v_end){
//			//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//		}
////		acc=0;
//		trapezoid_flg=0;
//	//				HAL_Delay(1000);
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_Delay(0);//300→10
////		HAL_Delay(500);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		//壁制御用のflgをオフ(0)にする
//		wall_control_flg=0;
//	 }



//20進んでいる間に歩数マップを作成するver.任意距離前進調整はタイヤ系で 壁制御あり
void step_ver_trapezoid_accel_forward(float a0,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
//	   acc=1000;//加速度の定義
//	   v_start=100;//初速定義
//	   v_max=500;//最高速度定義
//	   v_end=100;//終端速度定義
//	   x=540;//目標

	   //初期化
	   accm=a0;//externする用
	   acc=a0;
	   v_start=v0;
	   v_max=vM;
	   v_end=vE;
	   target_dis=tx;
	   dt=0.001;//刻み時間
	   dis=0;
	   left_dis=target_dis;
	   vel =v_start;
	   //壁制御オン(1)
	   wall_control_flg=1;


		trapezoid_flg=1;
//		motor_pwm_on();

//		x_dec = (vel*vel-v_end*v_end)/(2*a);
//		printf("%f\n\r",vel);
		while((vel < v_max)&&(left_dis>x_dec)){
//			step_number_revised();//走行中にキュー配列入りの歩数マップを作成する　while文中だと5回くらい繰り返されており、キュー配列がうまくいかないので、位置を変える
//			printf("Front_wall=%f,Right_wall=%f,Left_wall=%f\n\r",Front_wall,Right_wall,Left_wall);
//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//			Print_Wall_2();
		}
		if(vel > v_max){
			vel=vM;

		}
		acc=0;
		while(left_dis>x_dec){//
			step_number_revised();//走行中にキュー配列入りの歩数マップを作成する　while文中だと5回くらい繰り返されており、キュー配列がうまくいかないので、位置を変える

			//printf("%f\n\r",vel);
//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
		}
		acc=-a0;
		while(vel>v_end){
			//printf("%f\n\r",vel);
//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
		}
//		acc=0;
		trapezoid_flg=0;
	//				HAL_Delay(1000);
//		motor_pwm_off();//これがあると止まる止めないようにしたい

		//壁制御用のflgをオフ(0)にする
		wall_control_flg=0;
	 }

////初期段階のやつ
////20進んでいる間に歩数マップを作成するver.任意距離前進調整はタイヤ系で 壁制御あり
//void step_ver_trapezoid_accel_forward(float a0,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
////	   acc=1000;//加速度の定義
////	   v_start=100;//初速定義
////	   v_max=500;//最高速度定義
////	   v_end=100;//終端速度定義
////	   x=540;//目標
//
//	   //初期化
//	   accm=a0;//externする用
//	   acc=a0;
//	   v_start=v0;
//	   v_max=vM;
//	   v_end=vE;
//	   target_dis=tx;
//	   dt=0.001;//刻み時間
//	   dis=0;
//	   left_dis=target_dis;
//	   vel =v_start;
//	   //壁制御オン(1)
//	   wall_control_flg=1;
//
//	    HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		trapezoid_flg=1;
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
////		x_dec = (vel*vel-v_end*v_end)/(2*a);
////		printf("%f\n\r",vel);
//		while((vel < v_max)&&(left_dis>x_dec)){
//			step_number_revised();//走行中にキュー配列入りの歩数マップを作成する　while文中だと5回くらい繰り返されており、キュー配列がうまくいかないので、位置を変える
////			printf("Front_wall=%f,Right_wall=%f,Left_wall=%f\n\r",Front_wall,Right_wall,Left_wall);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
////			Print_Wall_2();
//		}
//		if(vel > v_max){
//			vel=vM;
//		}
//		acc=0;
//		while(left_dis>x_dec){//
//			//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//		}
//		acc=-a0;
//		while(vel>v_end){
//			//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//		}
////		acc=0;
//		trapezoid_flg=0;
//	//				HAL_Delay(1000);
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_Delay(0);//300→100にした→50 0にしてみた
////		HAL_Delay(500);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		//壁制御用のflgをオフ(0)にする
//		wall_control_flg=0;
//	 }

//backwardはhipdropの時くらいしか使わないと考えて励磁、PWM諸々をそのままにしている
void trapezoid_accel_backward(float a0,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離　//backwardはhipdropの時くらいしか使わないと考えて励磁、PWM諸々をそのままにしている
//	   acc=1000;//加速度の定義
//	   v_start=100;//初速定義
//	   v_max=500;//最高速度定義
//	   v_end=100;//終端速度定義
//	   x=540;//目標

	   //初期化
	   acc=-a0;
	   v_start=-v0;
	   v_max=-vM;
	   v_end=-vE;
	   target_dis=-tx;
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
//		x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
		while((fabs(vel) < fabs(v_max))&&(fabs(left_dis)>fabs(x_dec))){
		}
		if(fabs(vel)>fabs(v_max)){
			vel=-vM;
		}
		acc=0;
		while(fabs(left_dis)>fabs(x_dec)){
//			printf("%f\n\r",vel);
		}
		acc=a0;
		while(fabs(vel)>fabs(v_end)){
//			printf("%f\n\r",vel);
		}
//		acc=0;
		trapezoid_flg=0;
	//				HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(300);
//		HAL_Delay(500);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		//壁制御用のflgをオフ(0)にする
	    wall_control_flg=0;
	 }

//励磁、PWM諸々をそのままにしている
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

	   wall_control_flg=0;//壁制御 多分ここでは不要　超信地旋回の時は一旦壁制御切っても良いのでは

//		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		trapezoid_angle_flg=1;
		angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);
		while(angle_vel < angle_v_max){
			//printf("%f\n\r",angle);
		}
		angle_acc=0;
		while(target_angle-angle > angle_dec){
			//printf("%f\n\r",angle);
		}
		angle_acc=-angle_a;
		while(angle_vel>angle_v_end){
			//printf("%f\n\r",angle);
		}
		angle_acc=0;
		trapezoid_angle_flg=0;
//				HAL_Delay(1000);
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		wait_ms(300);
		//壁制御用のflgをオフ(0)にする
		//wall_control_flg=0;
}

//三角加速
//<三角>加速
//	 		 	  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//	 		 		   acc=1000;//加速度の定義
//	 				   v_start=100;//初速定義
//	 				   v_max=500;//最高速度定義
//	 				   v_end=100;//終端速度定義
//	 				   x=540;//目標
//	 				   dt=0.001;//刻み時間
//	 				   dis=0;
//	 				   vel =100;
//	 				HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	 				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//	 				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//	 				HAL_Delay(3);
//	 				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//	 				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	 				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	 				trapezoid_flg=1;
//	 				x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
//	 				while(vel < v_max){
//	  					printf("%f\n\r",vel);
//	  					while(x-dis>x_dec){
//	  						printf("%f\n\r",vel);
//	  									}
//	  					acc=-1000;
//	  				}
//	 				acc=0;
//	 				while(x-dis>x_dec){
//	 					printf("%f\n\r",vel);
//	 				}
//	 				acc=-1000;
//	 				while(vel>v_end){
//	 					printf("%f\n\r",vel);
//	 				}
//	 				acc=0;
//	 				trapezoid_flg=0;
//	 //				HAL_Delay(1000);
//	 				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//	 				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	 				HAL_Delay(500);
//	 				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//	 		 	 }

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
	   target_angle=-(angle_t);//目標回転角度//左と右で回転角度違う(右の方が大きい)ので角度を小さくしたい時は減ずる値(10)を大きくする//1228というのを改善した//元に戻すことで改善
	   dt=0.001;//刻み時間
	   angle=0;//変数としての角度
	   angle_vel =angle_v_start;//変数としての角速度

	   wall_control_flg=0;//壁制御 多分ここでは不要　超信地旋回の時は一旦壁制御切っても良いのでは

//		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		trapezoid_angle_flg=1;
		angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);

		while(fabs(angle_vel) < fabs(angle_v_max)){//fabsが中途半端な回転の原因になっているかも 試しに無くしてみる 条件文内のangle系の変数は全てfabsが元々ついていた
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
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		wait_ms(500);
		//壁制御用のflgをオフ(0)にする
		//wall_control_flg=0;
}

//void trapezoid_accel_forward(float a0,float v0,float vM,float vE,float tx){//台形加速左から加速度,初速,最大速度,終端速度,設定距離
////	   acc=1000;//加速度の定義
////	   v_start=100;//初速定義
////	   v_max=500;//最高速度定義
////	   v_end=100;//終端速度定義
////	   x=540;//目標
//
//	   //初期化
//	   accm=a0;//externする用
//	   acc=a0;
//	   v_start=v0;
//	   v_max=vM;
//	   v_end=vE;
//	   target_dis=tx;
//	   dt=0.001;//刻み時間
//	   dis=0;
//	   left_dis=target_dis;
//	   vel =v_start;
//	   //壁制御オン(1)
//	   wall_control_flg=1;
//
//	    HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		trapezoid_flg=1;
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
////		x_dec = (vel*vel-v_end*v_end)/(2*a);
////		printf("%f\n\r",vel);
//		while((vel < v_max)&&(left_dis>x_dec)){
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//
//		}
//		if(vel > v_max){
//			vel=vM;
//		}
//		acc=0;
//		while(left_dis>x_dec){//
//			//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//		}
//		acc=-a0;
//		while(vel>v_end){
//			//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//		}
////		acc=0;
//		trapezoid_flg=0;
//	//				HAL_Delay(1000);
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_Delay(0);//300→10
////		HAL_Delay(500);
//		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		//壁制御用のflgをオフ(0)にする
//		wall_control_flg=0;
//	 }
//

void slalom_trapezoid_accel_lturn(float center_of_gravity_vel,float angle_a,float angle_v0,float angle_vM,float angle_vE,float angle_t){//重心速度 ,角加速度,初角速,最大角速度,終端角速度,設定角度
	   float a0=2000;
       float v0=center_of_gravity_vel;
       float vM=center_of_gravity_vel;
       float vE=center_of_gravity_vel;
       float tx=105;
	   //初期化スラロームver.
	   accm=a0;//externする用
	   acc=a0;//加速度の定義
	   v_start=v0;//初速定義
	   v_max=vM;//最高速度定義
	   v_end=vE;//終端速度定義
	   target_dis=tx;//目標
	   dt=0.001;//刻み時間
	   dis=0;
	   left_dis=target_dis;
	   vel =v_start;
	   //壁制御オン(1)//一回なしで
//	   wall_control_flg=1;

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

	   wall_control_flg=0;//壁制御 多分ここでは不要　超信地旋回(スラロームも)の時は一旦壁制御切っても良いのでは

		slalom_trapezoid_flg=1;

		angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);

		while(angle_vel < angle_v_max){//直進速度は一定なので、台形加速の加速、減速条件は抜けよ
		//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);

				}

//		while(angle_vel < angle_v_max){
//			//printf("%f\n\r",angle);
//		}

		//

		//
		if(vel > v_max){
					vel=vM;
				}
				acc=0;

		angle_acc=0;

		while(target_angle-angle > angle_dec){//
					//printf("%f\n\r",vel);
		//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
				}
//				acc=-a0;

//		while(target_angle-angle > angle_dec){
//			//printf("%f\n\r",angle);
//		}
		angle_acc=-angle_a;

		while(angle_vel>angle_v_end){
					//printf("%f\n\r",vel);
		//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
				}
		//		acc=0;
//				trapezoid_flg=0;

//		while(angle_vel>angle_v_end){
//			//printf("%f\n\r",angle);
//		}
		angle_acc=0;
//		trapezoid_angle_flg=0;
		slalom_trapezoid_flg=0;

		//壁制御用のflgをオフ(0)にする
		wall_control_flg=0;
}

void slalom_trapezoid_accel_rturn(float center_of_gravity_vel,float angle_a,float angle_v0,float angle_vM,float angle_vE,float angle_t){//重心速度 ,角加速度,初角速,最大角速度,終端角速度,設定角度
	   float a0=2000;
       float v0=center_of_gravity_vel;
       float vM=center_of_gravity_vel;
       float vE=center_of_gravity_vel;
       float tx=105;
	   //初期化スラロームver.
	   accm=a0;//externする用
	   acc=a0;//加速度の定義
	   v_start=v0;//初速定義
	   v_max=vM;//最高速度定義
	   v_end=vE;//終端速度定義
	   target_dis=tx;//目標
	   dt=0.001;//刻み時間
	   dis=0;
	   left_dis=target_dis;
	   vel =v_start;
	   //壁制御オン(1)//一回なしで
//	   wall_control_flg=1;

//	   //以下角速度系初期化
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
	   target_angle=-angle_t;//目標 180°
	   dt=0.001;//刻み時間
	   angle=0;//変数としての角度
	   angle_vel =angle_v_start;//変数としての角速度

	   wall_control_flg=0;//壁制御 多分ここでは不要　超信地旋回(スラロームも)の時は一旦壁制御切っても良いのでは

		slalom_trapezoid_flg=1;

		angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);

		while(fabs(angle_vel) < fabs(angle_v_max)){//直進速度は一定なので、台形加速の加速、減速条件は抜けよ
		//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);

				}

//		while(angle_vel < angle_v_max){
//			//printf("%f\n\r",angle);
//		}

		//

		//
		if(vel > v_max){
					vel=vM;
				}
				acc=0;

		angle_acc=0;

		while(fabs(target_angle)-fabs(angle) > fabs(angle_dec)){//
					//printf("%f\n\r",vel);
		//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
				}
//				acc=-a0;

//		while(target_angle-angle > angle_dec){
//			//printf("%f\n\r",angle);
//		}
		angle_acc=angle_a;

		while(fabs(angle_vel)>fabs(angle_v_end)){
					//printf("%f\n\r",vel);
		//			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
				}
		//		acc=0;
//				trapezoid_flg=0;

//		while(angle_vel>angle_v_end){
//			//printf("%f\n\r",angle);
//		}
		angle_acc=0;
//		trapezoid_angle_flg=0;
		slalom_trapezoid_flg=0;

		//壁制御用のflgをオフ(0)にする
		wall_control_flg=0;
}

