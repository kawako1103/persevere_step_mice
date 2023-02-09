/*
 * mode.c
 *
 *  Created on: 2023/02/08
 *      Author: kawaguchitakahito
 */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "pl_timer.h"
#include "PL_lcd.h"
#include "PL_sensor.h"
#include "trapezoid_acc_model.h"
#include "Motor_Run.h"
#include "log.h"
#include "wall_control.h"
#include "explore_method.h"
#include "shortest_run.h"
//モードの設定
//液晶に表示させるやつ

// /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_USART2_UART_Init();
//  MX_TIM6_Init();
//  MX_TIM15_Init();
//  MX_I2C1_Init();
//  MX_ADC1_Init();
//  MX_TIM1_Init();
//  MX_TIM2_Init();


void warming_up(void){
	 pl_lcd_clear();
	 pl_timer_init();
	 pl_lcd_puts("be careful");//最短走行速度400()
	 pl_lcd_pos(1,0);
	 pl_lcd_puts("warm up");
	 HAL_Delay(1000);
	 ///////スタートの動き ウォーミングアップ的な
		  motor_excitation_on();
		  motor_pwm_on();
//					  HAL_Delay(500);//0.5秒経ってからスタート
		  trapezoid_accel_backward(600,100,300,100,80);//90back台形加速の関数遅くね
		  HAL_Delay(500);//
		  motor_pwm_off();

		  motor_pwm_on();
		  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
		  motor_pwm_off();
		  motor_excitation_off();
     /////////スタートの動き
}

void before_start_count(){
	 pl_lcd_clear();
	 pl_timer_init();
    pl_lcd_puts("ready?");
	 pl_lcd_pos(1,0);
	 pl_lcd_puts("500");
	 HAL_Delay(500);
	 pl_lcd_clear();
	 HAL_Delay(1000);//1秒経ってからスタート
	 pl_lcd_clear();
	 pl_timer_init();
	 pl_lcd_puts("3");//最短走行速度500()
	 pl_lcd_pos(1,0);
//				 pl_lcd_puts("500");
	 HAL_Delay(1000);
	 pl_lcd_clear();
	 pl_timer_init();
	 pl_lcd_puts("2");//最短走行速度500()
	 pl_lcd_pos(1,0);
	 HAL_Delay(1000);//1秒経ってからスタート
	 pl_lcd_clear();
	 pl_timer_init();
	 pl_lcd_puts("1");//最短走行速度500()
	 pl_lcd_pos(1,0);
	 HAL_Delay(1000);//1秒経ってからスタート
	 pl_lcd_clear();
	 pl_timer_init();
}


void mode_setting(){
	HAL_Delay(1000);
// if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)||(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){//SW1もしくはSW2を押したら設定画面へ
	 while(1){//設定画面 初期設定はスラローム探索速度500
//		 HAL_Delay(1000);
		 //SW1を押したら他の選択肢(探索スラローム単体か　最短走行のスラロームか、、)
		 if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
			 while(1){
				 //SW2だけ押されたら連続足立法
				 if ((HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)&&(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)!=0)){
					 warming_up();


					 while(1){
						 if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
					 			HAL_Delay(500);
								 pl_lcd_clear();
								 pl_timer_init();
								 pl_lcd_puts("exploring...");//
								 pl_lcd_pos(1,0);
					 			continual_adachi_method();//連続足立法
								 pl_lcd_clear();
								 pl_timer_init();
								 pl_lcd_puts("End");//
								 pl_lcd_pos(1,0);
					 			break;
					 			}
					 }
					 //TBD
				 //SW1とSW2どちらも押されたらもどる
				 }else if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)&&(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){

					 break;
				 }
			 pl_lcd_clear();
			 pl_timer_init();
			 pl_lcd_puts("ex_superstrike");//超信地旋回する連続足立法(連続ではない足立法は一旦なしで)
			 pl_lcd_pos(1,0);
			 pl_lcd_puts(" 500");
			 HAL_Delay(500);
			 }


		 }//SW2を押したら決定で探索開始(探索スラローム単体か　最短走行のスラロームか、、)
		 else if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
			 warming_up();
//			 pl_lcd_clear();
//			 pl_timer_init();
//			 pl_lcd_puts("be careful");//最短走行速度400()
//			 pl_lcd_pos(1,0);
//			 pl_lcd_puts("warm up");
//			 HAL_Delay(1000);
//			 ///////スタートの動き ウォーミングアップ的な
//				  motor_excitation_on();
//				  motor_pwm_on();
//	//					  HAL_Delay(500);//0.5秒経ってからスタート
//				  trapezoid_accel_backward(600,100,300,100,80);//90back台形加速の関数遅くね
//				  HAL_Delay(500);//
//				  motor_pwm_off();
//
//				  motor_pwm_on();
//				  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
//				  motor_pwm_off();
//				  motor_excitation_off();
//	          /////////スタートの動き
					 //ウォーミングアップ終わったらボタンを押して探索開始　その後次のwhile分へ
					 while(1){
					 if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
						 HAL_Delay(500);
						 pl_lcd_clear();
						 pl_timer_init();
						 pl_lcd_puts("exploring...");//
						 pl_lcd_pos(1,0);
						 slalom_continual_adachi_method();//スラローム付き連続足立法
						 pl_lcd_clear();
						 pl_timer_init();
						 pl_lcd_puts("End");//
						 pl_lcd_pos(1,0);
						 break;
					 }
					 }

			 //終了したら
			 while(1){
			 if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
				 while(1){
					 if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
						 HAL_Delay(500);
						 break;
					 }else if((HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){
						 pl_lcd_clear();
						 pl_timer_init();
						 pl_lcd_puts("short_slalom");//最短走行速度400()
						 pl_lcd_pos(1,0);
						 pl_lcd_puts("Sorry");
						 HAL_Delay(500);
					 }

					 pl_lcd_clear();
					 pl_timer_init();
					 pl_lcd_puts("short_slalom");//最短走行速度400()
					 pl_lcd_pos(1,0);
					 pl_lcd_puts("400");
					 HAL_Delay(500);
				 }

			 }else if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
				 before_start_count();

				 pl_lcd_puts("Go!!!!");//最短走行速度500()
				 pl_lcd_pos(1,0);
				 pl_lcd_puts("500");
				 after_explore_shortes_run(500);
				 }
			 pl_lcd_clear();
			 pl_timer_init();
			 pl_lcd_puts("short_slalom");//最短走行速度500()
			 pl_lcd_pos(1,0);
			 pl_lcd_puts("500");
			 HAL_Delay(500);
			 pl_lcd_clear();
			 }
//			 if(){
//
//			 }

		 }

	/////
	////
    ////
//		 while(1){//最初探索スラロームを提案(速度は一定500でよかろう　速度は表示しない)
//			 //SW2を押したらまず決定で探索開始(探索スラローム単体か　最短走行のスラロームか、、)
//			 if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//
//				 while(1){
//
//					//両方のボタンを押したらもどるのつもり
//					if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)&&(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){
//
//						break;
//
//
//					}//SW1を押したら他の選択肢へ
//					else if((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)){//最短走行のスラローム速度500の選択
//						pl_timer_init();
//						pl_lcd_puts("short_slalom");
//						pl_lcd_pos(1,0);
//						pl_lcd_puts(" 500");
//						HAL_Delay(500);
//						pl_lcd_clear();
//					}
//					//SW1を押したら他の選択肢へ
//					else if((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)){//最短走行のスラローム速度1000の選択
//						pl_timer_init();
//						pl_lcd_puts("short_slalom");
//						pl_lcd_pos(1,0);
//						pl_lcd_puts(" 1000");
//						HAL_Delay(500);
//						pl_lcd_clear();
//					}
//
//
//
//
//
//
//
//					pl_timer_init();
//				    pl_lcd_puts("ex_slalom");
////				    pl_lcd_pos(1,0);
////				    pl_lcd_puts(" 500");
//				    HAL_Delay(500);
//				    pl_lcd_clear();
//
//				 }
//			 }
//			 else if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//				   pl_timer_init();
//				   pl_lcd_puts("ex_slalom");
//				   pl_lcd_pos(1,0);
//				   pl_lcd_puts(" 500");
//				   HAL_Delay(500);
//				   pl_lcd_clear();
//			 }
//		   pl_timer_init();
//		   pl_lcd_puts("ex_slalom");
////		   pl_lcd_pos(1,0);
////		   pl_lcd_puts(" 500");
//		   HAL_Delay(500);
//		   pl_lcd_clear();
//
//		 }
		 ///////
		 //////
		 /////
		 pl_lcd_clear();
		 pl_timer_init();
		 pl_lcd_puts("ex_slalom");
	     HAL_Delay(500);
	     pl_lcd_clear();

//	 			if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//	 				break;
//	 			}

		}
}



//	if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//		while(1){
//			if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//
//			}
//
//		}
//	}
//						  HAL_Delay(500);//0.5秒経ってからスタート
//	//					  left_hand_method_and_Print_Wall();
//	//					  left_hand_method();
//	//					  left_hand_method_2();//左手法　足立法と一緒に絶対使うな
//	//					  adachi_method();//足立法
//						  //continual_adachi_method();//連続足立法
//						  slalom_continual_adachi_method();//スラローム付き連続足立法
//	//					  motor_excitation_on();
//	//					  motor_pwm_on();
//	//					  trapezoid_accel_forward(2000,100,500,100,90);//70む台形加速の関数 連続足立法ね
//	//					  motor_pwm_off();
//	//					  HAL_Delay(1000);//1秒経ってから励磁解除
//	//					  motor_excitation_off();
//	//					  step_number();//歩数マップ展開かな
////					  }
//}


