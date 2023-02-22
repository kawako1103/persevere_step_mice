/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#include "mode.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  pl_timer_init();
  pl_lcd_puts("Hello");
  pl_lcd_pos(1,0);
  pl_lcd_puts(" STM32");
  HAL_Delay(500);
  pl_lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  pl_lcd_init();
  pl_lcd_puts("Hello");
  pl_lcd_pos(1, 0);
  pl_lcd_puts("   STM32");
  HAL_Delay(500);
  pl_lcd_clear();
  pl_lcd_pos(0, 0);
  pl_lcd_puts("Mice");
  pl_lcd_pos(1, 0);
  pl_lcd_puts("ﾀｽｹﾃ");
  HAL_Delay(100);
  if(pl_getbatt() < LIPO_LIMIT){
	  pl_lcd_clear();pl_lcd_pos(0, 0);
	  pl_lcd_puts("LIPO");
	  pl_lcd_pos(1, 0);
	  pl_lcd_puts("error");
	  HAL_Delay(500);}

   int cnt=0;
   int g_ADCBuffer=0;
   double g_V_batt =0;
//   int trapezoid_flag;
   int i;
//   float x_dec=0;//trapezoid関数化に伴い不要かな
//   float target_dis=540;


   HAL_TIM_Base_Start_IT(&htim1);//motor
   HAL_TIM_PWM_MspInit(&htim1);//motor

   HAL_TIM_Base_Start_IT(&htim2);//motor
   HAL_TIM_PWM_MspInit(&htim2);//motor

   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  while (1)
  {
//	  //test LED2(Lチカ)
//	  HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	  HAL_Delay(1000);
//
//	  HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_RESET);
//	  HAL_Delay(1000);

	  //test SWITCH_1,2(スイッチ1でLEDが点いて、スイッチ2でLEDが消える)
//	  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//	  HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	  HAL_Delay(100);
//	  }
//	  if(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//	  HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_RESET);
//	  HAL_Delay(100);
//	  }

	  //test USART1(tera termにHello Worldが送られる)
	  //uint8_t hello[] = "Hello World\n\r";
	  //HAL_UART_Transmit(&huart2, hello, sizeof(hello),1000);
	  //test USART2
//	  uint8_t hello[] = "Hello World\n\r";
//	  float PI=3.14;
//	  setbuf(stdout, NULL);
//	  printf("hello=%s",hello);
//	  uint8_t hoge = 3;
//	  printf("hoge=%d\n\r", hoge);
//	  printf("M_PI=%f\n\r", PI);


////	  //test timer
//	  HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//	  wait_ms(3000);
//	  HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_RESET);
//	  wait_ms(3000);

//	  // test speaker
//	 if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//		HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		HAL_Delay(1000);
//		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
//		while(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==1){
//		}
//		HAL_Delay(100);
//		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
//	 }
	 // test LCD
//	 	 char strBuffer[17] = {0};
//	 	 sprintf(strBuffer, "CNT=%04d", cnt);
//	 	 cnt++;
//	 	 pl_lcd_pos(1, 0);
//	 	 pl_lcd_puts(strBuffer);

////閾値とかはここでセンサー//sensor test
//		 HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
//		 printf("BATT=%f\n\r",g_V_batt);
//		 printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d\n\r", g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
//		 wait_ms(500);

		  //motor test
//		 	  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//		     	HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//		      	HAL_Delay(1000);
//		 		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//		 		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//		 		HAL_Delay(3);
//		 		HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//		 		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		 		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//		 		HAL_Delay(5000);
//		 	 	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		 	 	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		 	 	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		 	 	HAL_Delay(1000);
//		 	  }
		  //<台形>加速の概要
		 //台形加速関数化 試行
//		 		   acc=1000;//加速度の定義
//				   v_start=100;//初速定義
//				   v_max=500;//最高速度定義
//				   v_end=100;//終端速度定義
//				   x=540;//目標
//				   dt=0.001;//刻み時間
//				   dis=0;
//				   vel =100;
//		 if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//		 trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速(壁制御あり)の関数
////
////	     //2周テスト用
////		 trapezoid_accel_lturn(1000,100,400,80,90);//左90°曲がる
////////		 trapezoid_accel_lturn(1000,100,400,80,90);//90°曲がる
////////         trapezoid_accel_lturn(1000,100,400,80,90);//90°曲がる
////////         trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////////         trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////////		 trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////////		 trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////////		 trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
//////
////		 trapezoid_accel_rturn(1350,100,400,80,90);//右90°曲がる//曲がりすぎるのでちょっと加速度を大きく
//		 }

         //	           angle_acc=1000;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
		 //			   angle_v_start=100;//初角速度定義
		 //			   angle_v_max=400;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
		 //			   angle_v_end=80;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
		 //			   target_angle=180;//目標 180°

//		// <台形>加速の概要 型とか初期化とかミスらなければちゃんと動く
//		 	  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//		 		   acc=1000;//加速度の定義
//				   v_start=100;//初速定義
//				   v_max=500;//最高速度定義
//				   v_end=100;//終端速度定義
//				   target_dis=270;//目標
//				   dt=0.001;//刻み時間
//				   dis=0;//変数として
//				   vel =100;
//				HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//				HAL_Delay(3);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//				trapezoid_flg=1;
//				x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
//				while(vel < v_max){
//				}
//				acc=0;
//				while(target_dis-dis>x_dec){
//					printf("%f\n\r",vel);
//				}
//				acc=-1000;
//				while(vel>v_end){
//					printf("%f\n\r",vel);
//				}
//				acc=0;
//				trapezoid_flg=0;
////				HAL_Delay(1000);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//		 	 }
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

  //<三角>加速の速度のlog取り
//			  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//			   acc=1000;//加速度の定義
//			   v_start=100;//初速定義
//			   v_max=500;//最高速度定義
//			   v_end=100;//終端速度定義
//			   x=540;//目標
//			   dt=0.001;//刻み時間
//			   dis=10;
//			   vel =100;
//				HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//				HAL_Delay(3);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//				log_vel_flg=1;
//				//以下三角加速
//				trapezoid_flg=1;
//				x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
//				while(vel < v_max){
////					printf("%f\n\r",vel);
//					while(x-dis>x_dec){
//					//					printf("%f\n\r",vel);
//									}
//					acc=-1000;
//				}
//				acc=0;
//				while(x-dis>x_dec){
////					printf("%f\n\r",vel);
//				}
//				acc=-1000;
//				while(vel>v_end){
////					printf("%f\n\r",vel);
//				}
//				acc=0;
//				trapezoid_flg=0;
////				HAL_Delay(1000);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//				//台形の加速終了
//				log_vel_flg=0;
//				for(i=0;i<=t;i++){
//					printf("%f\n\r",record[i]);
//				}
//			 }

//	  //<超信地旋回>の概要//これは安定してうまくいく
//		 	  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//		 	   //一応初期化
//		 	   acc=0;//加速度の定義
//			   v_start=0;//初速定義
//			   v_max=0;//最高速度定義
//			   v_end=0;//終端速度定義
//			   target_dis=0;//目標
//			   dis=0;
//			   vel =0;
//			   //以下角速度系初期化
//		 	   angle_acc=1000;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
//			   angle_v_start=100;//初角速度定義
//			   angle_v_max=400;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
//			   angle_v_end=80;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
//			   target_angle=180;//目標 180°
//			   dt=0.001;//刻み時間
//			   angle=0;//変数としての角度
//			   angle_vel =10;//変数としての角速度
//
//			   //wall_control_flg=1;//壁制御 超信地旋回の時は一旦壁制御切っても良いのでは
//
//				HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//				HAL_Delay(3);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//
//				trapezoid_angle_flg=1;
//				angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);
//				while(angle_vel < angle_v_max){
//					//printf("%f\n\r",angle);
//				}
//				angle_acc=0;
//				while(target_angle-angle > angle_dec){
//					//printf("%f\n\r",angle);
//				}
//				angle_acc=-2000;
//				while(angle_vel>angle_v_end){
//					//printf("%f\n\r",angle);
//				}
//				angle_acc=0;
//				trapezoid_angle_flg=0;
////				HAL_Delay(1000);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//				wait_ms(500);
//				//壁制御用のflgをオフ(0)にする
//				//wall_control_flg=0;
//
//				//2回目
//				//一応初期化
//						 	   acc=0;//加速度の定義
//							   v_start=0;//初速定義
//							   v_max=0;//最高速度定義
//							   v_end=0;//終端速度定義
//							   target_dis=0;//目標
//							   dis=0;
//							   vel =0;
//							   //以下角速度系初期化
//						 	   angle_acc=1000;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
//							   angle_v_start=100;//初角速度定義
//							   angle_v_max=400;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
//							   angle_v_end=80;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
//							   target_angle=180;//目標 180°
//							   dt=0.001;//刻み時間
//							   angle=0;//変数としての角度
//							   angle_vel =10;//変数としての角速度
//
//							   //wall_control_flg=1;//壁制御 超信地旋回の時は一旦壁制御切っても良いのでは
//
//								HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//								HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//								HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//								HAL_Delay(3);
//								HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//								HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//								HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//
//								trapezoid_angle_flg=1;
//								angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);
//								while(angle_vel < angle_v_max){
//									//printf("%f\n\r",angle);
//								}
//								angle_acc=0;
//								while(target_angle-angle > angle_dec){
//									//printf("%f\n\r",angle);
//								}
//								angle_acc=-2000;
//								while(angle_vel>angle_v_end){
//									//printf("%f\n\r",angle);
//								}
//								angle_acc=0;
//								trapezoid_angle_flg=0;
//				//				HAL_Delay(1000);
//								HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//								HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//								HAL_Delay(500);
//								HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//
//								//壁制御用のflgをオフ(0)にする
//								//wall_control_flg=0;
//
//		 	    }

//三角加速による超信地旋回
// 	  //<超信地旋回>の概要
//			  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//			   angle_acc=2000;//角加速度の定義//ここの調整がなかなか　回転はすぐしたいから傾き大きめで良さげ//三角角加速の方がいいかも
//			   angle_v_start=100;//初角速度定義
//			   angle_v_max=400;//最高角速度定義//(400^2-100^2)/(2*2000)=500*300/(2*2000)=15*10^4/4000=37.5°で加角速
//			   angle_v_end=80;//終端角速度定義 //(400^2-80^2)/(2*2000)=480*320/(2*2000)=120*2^5/1000=2^7*30/1000=3.840°で減角速
//			   target_angle=180;//目標 180°
//			   dt=0.001;//刻み時間
//			   angle=0;//変数としての角度
//			   angle_vel =10;//変数としての角速度
//
//			   wall_control_flg=1;//壁制御 超信地旋回の時は一旦壁制御切っても良いのでは
//
//				HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//				HAL_Delay(3);
//				HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//				trapezoid_angle_flg=1;
//				angle_dec = (angle_v_max*angle_v_max-angle_v_end*angle_v_end)/(2*angle_acc);
//				while(angle_vel < angle_v_max){
//					printf("%f\n\r",angle_vel);
//					while(target_angle-angle>angle_dec){
//								//printf("%f\n\r",angle_vel);
//														}
//					angle_acc=-1000;
//				}
//				angle_acc=0;
//				while(target_angle-angle > angle_dec){
//					printf("%f\n\r",angle_vel);
//				}
//				angle_acc=-1000;
//				while(angle_vel>angle_v_end){
//					printf("%f\n\r",angle_vel);
//				}
//				angle_acc=0;
//				trapezoid_angle_flg=0;
// //				HAL_Delay(1000);
//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
//
//				//壁制御用のflgをオフ(0)にする
//				wall_control_flg=0;
//			 }


////センサー取得なにこれ
//		  		 HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
//		  		 printf("BATT=%f\n\r",g_V_batt);
//		  		 printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d\n\r", g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
//		  		 wait_ms(500);
//		  		 HAL_Delay(1000);

//壁制御の概要1(2,3も共通かな)
//				  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//				   acc=1000;//加速度の定義
//				   v_start=100;//初速定義
//				   v_max=500;//最高速度定義
//				   v_end=100;//終端速度定義
//				   x=540;//目標
//				   dt=0.001;//刻み時間
//				   dis=0;
//				   vel =100;
//				   //壁制御用のflgをオン(1)にする
//				   wall_control_flg=1;
//					HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
//					HAL_Delay(3);
//					HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
//					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//					//以下台形加速
//					trapezoid_flg=1;
//					x_dec = (v_max*v_max-v_end*v_end)/(2*acc);
//					while(vel < v_max){
//	//					printf("%f\n\r",vel);
//						while(x-dis>x_dec){
//						//					printf("%f\n\r",vel);
//										}
//						acc=-1000;
//					}
//					acc=0;
//					while(x-dis>x_dec){
//	//					printf("%f\n\r",vel);
//					}
//					acc=-1000;
//					while(vel>v_end){
//	//					printf("%f\n\r",vel);
//					}
//					acc=0;
//					//台形の加速終了
//					trapezoid_flg=0;
//	//				HAL_Delay(1000);
//					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//					HAL_Delay(500);
//					HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
////					log_vel_flg=0;
////					for(i=0;i<=t;i++){
////						printf("%f\n\r",record[i]);
//					//壁制御用のflgをオフ(0)にする
//					wall_control_flg=0;
//
//					}

//<左手法>の概要/動作のみ
//				  float THERESHOULD_L=260;//左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100
//				  float THERESHOULD_R=180;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70
//				  float THERESHOULD_FL=140;//左壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140
//				  float THERESHOULD_FR=220;//右壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220
//				  if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  while(1){
//				  if((float)g_sensor[1][0]<THERESHOULD_L){//[1][0]は、左壁, [2][0]は、右壁 //ここは左壁無し
//				  //90 mm直進,90°左旋回,90 mm直進,停止
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//		 	 	  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//		 	 	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  }else if(g_sensor[0][0]<THERESHOULD_FL&&g_sensor[3][0]<THERESHOULD_FR){//前壁なし
////					g_WallControlStatus=g_WallControlStatus&~(1<<0);//1bit目を０にする
//				  //180 mm直進,停止
//				  trapezoid_accel_forward(2000,100,500,100,180);//90む台形加速の関数
//				  }else if((float)g_sensor[2][0]<THERESHOULD_R){//右壁がない
////					g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
//				  //90 mm直進,90°右旋回,90 mm直進,停止
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  trapezoid_accel_rturn(2000,100,400,80,90);//左90°曲がる
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  }else{
////					g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
//				  //90前進,180°左旋回,90前進,停止
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  trapezoid_accel_lturn(2000,100,400,80,180);//左90°曲がる
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  }
//				  }
//				  }

  //<左手法>の概要/動作＋壁情報保存
//				  float THERESHOULD_L=260;//左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100
//				  float THERESHOULD_R=180;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70
//				  float THERESHOULD_FL=260;//左壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140
//				  float THERESHOULD_FR=180;//右壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220
//				  int column[16];
//				  int row[16];
//				  int x=0;
//				  int y=0;
//				  int z=0;//(方角,z)=(北,0),(東,1),(南,2),(西,3),(-1になったら3に変換,4になったら0に変換あ(後で入れる))


//	  //閾値とかはここでセンサー//sensor test
	  		 //HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
//	  		 printf("BATT=%f\n\r",g_V_batt);
	  		 printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d\n\r", g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
	  		 wait_ms(500);



//スイッチ するときはモードの条件を外して
/////////////////////////////////////
	  ///////////////////////////////
	  ///////////モード設定用//////////
	  ///////////////////////////////
	  //モード設定の条件(同時に押されたら設定に入る)
	  if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)&&(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){
		  mode_setting();
	  }
/////////////////////////////////////
	  ///////////////////////////////
	  ///////////////////////////////
	  ///////////////////////////////
				  if (((HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0))&&((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)!=0))){
					  HAL_Delay(500);//0.5秒経ってからスタート
////					  left_hand_method_and_Print_Wall();
////					  left_hand_method();
////					  left_hand_method_2();//左手法　足立法と一緒に絶対使うな
////					  adachi_method();//足立法
//					  //continual_adachi_method();//連続足立法
//					  slalom_continual_adachi_method();//スラローム付き連続足立法
////					  motor_excitation_on();
////					  motor_pwm_on();
////					  trapezoid_accel_forward(2000,100,500,100,90);//70む台形加速の関数 連続足立法ね
////					  motor_pwm_off();
////					  HAL_Delay(1000);//1秒経ってから励磁解除
////					  motor_excitation_off();
////					  step_number();//歩数マップ展開かな
				motor_excitation_on();
				motor_pwm_on();
//	  	  	  	trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
//	  			motor_pwm_off();
//	  			motor_pwm_on();
//	  //			trapezoid_accel_backward(1000,100,300,100,80);
//	  //			motor_pwm_off();
//	  //			motor_pwm_on();
	  			trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
	  			motor_pwm_off();
	  			motor_pwm_on();
//
	  			trapezoid_accel_backward(2000,100,200,100,80);
	  			motor_pwm_off();
	  			motor_pwm_on();

	  			non_wall_control_trapezoid_accel_forward(2000,100,300,100,30);//ここで中心にこれているか　左右で傾いていないか
	  			motor_pwm_off();
	  			motor_pwm_on();
	  			trapezoid_accel_lturn(2000,100,400,80,90);//90°曲がる
	  			motor_pwm_off();
	  			motor_pwm_on();
	  			trapezoid_accel_backward(2000,100,200,100,80);
	  			motor_pwm_off();
	  			motor_pwm_on();
	  			trapezoid_accel_forward(2000,100,500,500,120);//ここで中心にこれているか　左右で傾いていないか
	  			motor_pwm_off();
	  			HAL_Delay(1000);//0.5秒経ってからスタート
	  			motor_excitation_off();

				  }
/////
////
////
				  if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)&&((HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)!=0))){
//					  Print_Wall();//ボタンを押したら出力
//					  Print_Wall_2();
					  HAL_Delay(500);//0.5秒経ってからスタート
//				  }

					  ////////
					  ////////
					  ///////スタートの動き ウォーミングアップ的な
//					  motor_excitation_on();
//					  motor_pwm_on();
//  //					  HAL_Delay(500);//0.5秒経ってからスタート
//					  trapezoid_accel_backward(600,100,300,100,80);//90back台形加速の関数遅くね
//					  HAL_Delay(500);//
//					  motor_pwm_off();
//
//					  motor_pwm_on();
//					  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
//					  motor_pwm_off();
//					  motor_excitation_off();
//					  					  ///////スタートの動き
//					  //////
//					  //////
//					  //////

//					  after_explore_shortes_run(500);

			  ///////////
			  ///////////最短走行のテスト用やつ
			  ///////////

//					  while(1){
//
//					  if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//						 pl_lcd_clear();
//						 pl_timer_init();
//						 pl_lcd_puts("short_run");//最短走行速度400()
//						 pl_lcd_pos(1,0);
//						 pl_lcd_puts("500");
//						 HAL_Delay(500);//0.5秒経ってからスタート
//
//						 after_explore_shortes_run(500);
//						 pl_lcd_clear();
//						 pl_timer_init();
//						 pl_lcd_puts("end");//最短走行速度400()
//						 pl_lcd_pos(1,0);
//						 pl_lcd_puts("nice run?");
//						 HAL_Delay(500);//0.5秒経ってからスタート
//						 while(1){
//							 if((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)){
//								 Print_Wall_2();
//							 }
//						 }
//					  }else if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
//						  break;
//					  }
////					  Print_Wall_2();
//
//					  }
//					  Print_Wall_2();
// }
				  ///////////
				  ///////////最短走行のテスト用やつ終わり
				  ///////////
/////////////////////////////////////////
				      /////
				      /////
				  	  ////////////////壁切れ補正のテスト用
//				      motor_excitation_on();
//				      motor_pwm_on();
//				      step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//				      wall_cut_detection_trapezoid_accel_forward(2000,500,500,500,160);
//				      motor_pwm_off();
//				      HAL_Delay(1000);//1秒経ってから励磁解除
//				      motor_excitation_off();
//				  }
					  ////
					  ////2700mm(15マス分の連続走行(20mm + 160mmを繰り返すver.)のテスト用)
					  ////
//					  motor_excitation_on();
//					  motor_pwm_on();
////
////
//					  step_ver_trapezoid_accel_forward(2000,100,500,500,20);
//					  trapezoid_accel_forward(2000,500,500,500,160);//70む台形加速の関数 連続足立法ね
//					  for(i=0;i<=3;i+=1){
//						  step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//						  trapezoid_accel_forward(2000,500,500,500,160);//70む台形加速の関数 連続足立法ね
//					  }
//					  step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//					  trapezoid_accel_forward(2000,500,500,100,160);//70む台形加速の関数 連続足立法ね
//					  motor_pwm_off();
//					  HAL_Delay(1000);//1秒経ってから励磁解除
//					  motor_excitation_off();
					  ////
					  ////2700mm(15マス分の連続走行(20mm + 160mmを繰り返すver.)のテスト用)終わり
					  ////
					  /////スタートの動き
					  motor_excitation_on();
					  motor_pwm_on();
//					  HAL_Delay(500);//0.5秒経ってからスタート
//					  trapezoid_accel_backward(600,100,300,100,80);//90back台形加速の関数遅くね
//					  HAL_Delay(500);//
//					  motor_pwm_off();
//
//					  motor_pwm_on();
					  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
					  /////スタートの動き

//					  ///スラローム調整用 1.5マス進んでslalomで右折 then 180進んで停止
//					  ////
					  motor_excitation_on();
					  motor_pwm_on();
					  trapezoid_accel_forward(2000,500,500,500,180);
					  step_ver_trapezoid_accel_forward(2000,500,500,500,20);
					  ////////
					  ////////横壁制御
					  float control_left_sensor;
					  float control_right_sensor;
					  float offset_adjustment_len_rslalom;
					  float offset_adjustment_len_lslalom;

//						if((float)g_sensor[1][0]>=WALLREAD_L){//左あり
//							control_left_sensor = g_sensor[1][0];//オフセットを調整するためのもの
////							offset_adjustment_len_lslalom =  -1*(0.000001)*pow(control_left_sensor,3)+0.0022*pow(control_left_sensor,2)-1.3589*control_left_sensor+309.07 + 37 ;
//							offset_adjustment_len_rslalom =  -7*(0.000001)*pow(control_left_sensor,3)+0.0068*pow(control_left_sensor,2)-2.4156*control_left_sensor+321.2 + 37 ;
//							///右に曲がるとき, 左センサ(センサ値x)に対しては左向きを正として y = ((37-0)/(901-595))*(x-595)+0 　その後, yだけ左によっているので, yだけオフセットを増やす 両符号
//							///R 0:599/30:485/60:275//74:178
//							///左に曲がるとき, 右センサ(センサ値x)に対しては右向きを正として y = ((37-0)/(599-388.5))*(x-388.5)+0
//						}else{
//							offset_adjustment_len_rslalom = 0;
//						}
//										offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,150,43);//r(500,10000,100,460,80,93,75)→; (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
						offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,97,79);///これでいく(20mm全身で左に5度くらい傾いているからあえて回転角度を下げている感じになっている)r(500,10000,100,460,80,98,68);//r(500,10000,100,460,80,93,75)→; (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//						offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,96.9,90.8);//2/17 20:16これでいく
						//slalom_trapezoid_accel_lturn(500,10000,100,460,80,91,105);//(500,10000,100,460,80,93,105);
						//スラロームは角度90°を角加速度によって合わせる　→　オフセットを(80のところ)を調整して中心線合わせる
//						//slalom_trapezoid_accel_rturn(500,17000,100,460,80,92,75);これは多分使わない調整ミスしてた
						non_wall_control_trapezoid_accel_forward(2000,500,500,500,38);//横壁制御分入れた
						//↓横壁制御用
//						non_wall_control_trapezoid_accel_forward(2000,500,500,500,20 + offset_adjustment_len_rslalom);//横壁制御分入れた
						////////
						////////
					  //////
					  //////横壁制御なしver.
					  //////
					  //slalom_trapezoid_accel_rturn(500,10000,100,460,80,91);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
					  //non_wall_control_trapezoid_accel_forward(2000,500,500,500,20);
					  non_wall_control_step_ver_trapezoid_accel_forward(2000,500,500,500,20);

					  ///壁制御なし
					  non_wall_control_trapezoid_accel_forward(2000,500,500,100,160);
					  ///////trapezoid_accel_forward(2000,500,500,100,160);
					  ///
					  					  ////step_ver_trapezoid_accel_forward(2000,500,500,500,20);
					  					  ////slalom_trapezoid_accel_rturn(500,15000,250,460,230,90);
					  					  ////step_ver_trapezoid_accel_forward(2000,500,500,100,20);
					  	  	  	  	  	  ////trapezoid_accel_forward(2000,500,500,100,20);
					  motor_pwm_off();
					  //
					  HAL_Delay(1000);//1秒経ってから励磁解除
					  motor_excitation_off();
				  }
					  ///スラローム終

//					  		  motor_pwm_off();
//					  		  motor_pwm_on();
//					  		  trapezoid_accel_rturn(2000,100,400,80,90);//右90°曲がる
//					  		  motor_pwm_off();
//					  		  motor_pwm_on();
//					  		  trapezoid_accel_forward(2000,100,500,500,90);//90む台形加速の関数
//					  		trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数

//					  slalom_trapezoid_accel_lturn(10000,100,460,80,90);//超信地旋回左から角加速度,初角速,最大角速度,終端角速度,設定角度
//					  motor_excitation_on();
//					  motor_pwm_on();
//
//
//					  step_ver_trapezoid_accel_forward(2000,100,500,500,20);
//					  trapezoid_accel_forward(2000,500,500,500,160);
//
//					  step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//					  trapezoid_accel_forward(2000,500,500,100,160);
////
//					  motor_pwm_off();
////
//					  HAL_Delay(1000);//1秒経ってから励磁解除
//					  motor_excitation_off();

//				  if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
////					  if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//						  trapezoid_accel_forward(2000,100,500,100,180);//90む台形加速の関数

//						  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//						  motor_pwm_off();
//						  motor_pwm_on();
//						  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//						  motor_pwm_off();
//						  motor_pwm_on();
//						  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//						  motor_pwm_off();
//						  motor_pwm_on();
//						  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//////						  trapezoid_accel_backward(2000,100,300,100,80);//90back台形加速の関数遅くね
//////					  }
//						  motor_pwm_off();
//						 //
//						 					  HAL_Delay(1000);//1秒経ってから励磁解除
//						 					  motor_excitation_off();
//				  }

//				  if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//					  wall_information_initialize();
//					  step_number();//歩数マップ
//
//				  }
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  x=x;
//				  y=y+1;
//				  z=z;//　最初(0,0)にいた、(この時、(0,0.5)あたりにいるが一旦(0,0)にいたとみなす)北向き
//				  //
//				  //座標(いたマスの座標→センサーの捉える壁の座標)と方角更新→壁情報更新(if文で方角によって座標→センサーの手筈を決める)
//				  //動作(動作直前の条件分布は何も見ないこと)によって座標だけ更新1マスしか進まない//動作によって方角と座標が1つ変わる(直進なら変わらないが)
//				  while(1){
//					  if(x==1&&y==1){
//							trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//							break;//無限ループから脱出
//									}
//				  if((float)g_sensor[1][0]<THERESHOULD_L){//[1][0]は、左壁, [2][0]は、右壁 //ここは左壁無し
//				  //90 mm直進,90°左旋回,90 mm直進,停止
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  //座標更新
//				  if(z==0||z==4){//北に対して、、
//					  x=x-1;
//					  y=y;
//					  if(z==4){
//						  z=0;
//					  }
//				  }
//				  if(z==1){//東に対して、、
//					  x=x;
//					  y=y+1;
//				  }
//				  if(z==2){//南に対して、、
//					  x=x+1;
//					  y=y;
//				  }
//				  if(z==3){//西に対して、、
//					  x=x;
//					  y=y-1;
//					  if(z==-1){
//						  z=3;
//					  }
//				  }
//				  //向き更新
//					  z=z-1;
//				  }else if(g_sensor[0][0]<THERESHOULD_FL&&g_sensor[3][0]<THERESHOULD_FR){//前壁なし
//				  //180 mm直進,停止
//				  trapezoid_accel_forward(2000,100,500,100,180);
//				  //座標更新
//				  if(z==0||z==4){//北に対して、、
//					  x=x;
//					  y=y+1;
//					  if(z==4){
//						  z=0;
//					  }
//				  }
//				  if(z==1){//東に対して、、
//					  x=x+1;
//					  y=y;
//				  }
//				  if(z==2){//南に対して、、
//					  x=x;
//					  y=y-1;
//				  }
//				  if(z==3){//西に対して、、
//					  x=x-1;
//					  y=y;
//					  if(z==-1){
//						  z=3;
//					  }
//				  }
//				  //向き更新
//					  z=z;
//				  }else if((float)g_sensor[2][0]<THERESHOULD_R){//右壁がない
//	//					g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
//				  //90 mm直進,90°右旋回,90 mm直進,停止
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  trapezoid_accel_rturn(2000,100,400,80,90);//左90°曲がる
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  //座標更新
//				  if(z==0||z==4){//北に対して、、
//					  x=x+1;
//					  y=y;
//					  if(z==4){
//						  z=0;
//					  }
//				  }
//				  if(z==1){//東に対して、、
//					  x=x;
//					  y=y-1;
//				  }
//				  if(z==2){//南に対して、、
//					  x=x-1;
//					  y=y;
//				  }
//				  if(z==3){//西に対して、、
//					  x=x;
//					  y=y+1;
//					  if(z==-1){
//						  z=3;
//					  }
//				  }
//				  //向き更新
//					  z=z+1;
//
//				  }else{
//	//					g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
//				  //90前進,180°左旋回,90前進,停止
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  trapezoid_accel_lturn(2000,100,400,80,180);//左90°曲がる
//				  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//				  //座標更新
//				  if(z==0||z==4){//北に対して、、
//					  x=x;
//					  y=y-1;
//					  if(z==4){
//						  z=0;
//					  }
//				  }
//				  if(z==1){//東に対して、、
//					  x=x-1;
//					  y=y;
//				  }
//				  if(z==2){//南に対して、、
//					  x=x;
//					  y=y+1;
//				  }
//				  if(z==3){//西に対して、、
//					  x=x+1;
//					  y=y;
//					  if(z==-1){
//						  z=3;
//					  }
//				  }
//				  //向き更新
//					  z=z-1;
//				  }
//
//				  //方角によってどう変わるか//壁情報は今までいた座標によって決まる
//				  if(z==0||z==4){//(x,y),北に対して、、
//					  //左のセンサー
//					  if((float)g_sensor[1][0]<THERESHOULD_L){//左なし
//						  column[x-1]=column[x-1]&~(1<<y+1);//y+1ビット目を0にする
//					  }else{//左あり
//						  column[x-1]=column[x-1]|(1<<y+1);//y+1ビット目を1にする
//					  }
//					  //前のセンサー
//					  if(g_sensor[0][0]<THERESHOULD_FL&&g_sensor[3][0]<THERESHOULD_FR){//前なし
//						  row[y]=row[y]&~(1<<x+1);//x+1ビット目を0にする
//					  }else{//前あり
//						  row[y]=row[y]|(1<<x+1);//x+1を1にする
//					  }
//					  //右のセンサー
//					  if((float)g_sensor[2][0]<THERESHOULD_R){//右がない
//						  column[x]=column[x]&~(1<<y+1);//y+1ビット目を0にする
//					  }else{//右あり
//						  column[x]=column[x]|(1<<y+1);//y+1ビット目を1にする
//					  }
//
//					  if(z==4){
//						  z=0;
//					  }
//				  }else if(z==1){//(x,y),東に対して、、
//					  //左のセンサー
//					  if((float)g_sensor[1][0]<THERESHOULD_L){//左なし
//						  row[y]=row[y]&~(1<<x+1);//x+1ビット目を0にする
//					  }else{//左あり
//						  row[y]=row[y]|(1<<x+1);//x+1を1にする
//					  }
//					  //前のセンサー
//					  if(g_sensor[0][0]<THERESHOULD_FL&&g_sensor[3][0]<THERESHOULD_FR){//右なし
//
//						  column[x]=column[x]&~(1<<y+1);//y+1ビット目を0にする
//					  }else{//右あり
//						  column[x]=column[x]|(1<<y+1);//y+1ビット目を1にする
//					  }
//					  //右のセンサー
//					  if((float)g_sensor[2][0]<THERESHOULD_R){//右がない
//						  row[y-1]=row[y-1]&~(1<<x+1);//x+1ビット目を0にする
//					  }else{//右あり
//						  row[y-1]=row[y-1]|(1<<x+1);//x+1を1にする
//					  }
//
//				  }else if(z==2){//(x,y),南に対して、、
//					  //左のセンサー
//					  if((float)g_sensor[1][0]<THERESHOULD_L){//左なし
//						  column[x]=column[x]&~(1<<y+1);//y+1ビット目を0にする
//					  }else{//左あり
//						  column[x]=column[x]|(1<<y+1);//y+1ビット目を1にする
//					  }
//					  //前のセンサー
//					  if(g_sensor[0][0]<THERESHOULD_FL&&g_sensor[3][0]<THERESHOULD_FR){//前なし
//						  row[y-1]=row[y-1]&~(1<<x+1);//x+1ビット目を0にする
//					  }else{//前あり
//						  row[y-1]=row[y-1]|(1<<x+1);//x+1を1にする
//					  }
//					  //右のセンサー
//					  if((float)g_sensor[2][0]<THERESHOULD_R){//右がない
//						  column[x-1]=column[x-1]&~(1<<y+1);//y+1ビット目を0にする
//					  }else{//右あり
//						  column[x-1]=column[x-1]|(1<<y+1);//y+1ビット目を1にする
//					  }
//
//				  }else if(z==3||z==-1){//(x,y),西に対して、、
//					  //左のセンサー
//					  if((float)g_sensor[1][0]<THERESHOULD_L){//左なし
//						  row[y-1]=row[y-1]&~(1<<x+1);//x+1ビット目を0にする
//					  }else{//左あり
//						  row[y-1]=row[y-1]|(1<<x+1);//x+1を1にする
//					  }
//					  //前のセンサー
//					  if(g_sensor[0][0]<THERESHOULD_FL&&g_sensor[3][0]<THERESHOULD_FR){//右なし
//
//						  column[x]=column[x]&~(1<<y+1);//y+1ビット目を0にする
//					  }else{//右あり
//						  column[x]=column[x]|(1<<y+1);//y+1ビット目を1にする
//					  }
//					  //右のセンサー
//					  if((float)g_sensor[2][0]<THERESHOULD_R){//右がない
//						  row[y]=row[y]&~(1<<x+1);//x+1ビット目を0にする
//					  }else{//右あり
//						  row[y]=row[y]|(1<<x+1);//x+1を1にする
//					  }
//					  if(z==-1){
//						  z=3;
//					  }
//				  }
//
//				  }
//				  }

//				  //壁情報の打ち出し
//				  void Print_Wall();
//				  }
//				  	int x,xx,y,i;
//				  	printf("Print Start\n");
//				  		for(y=15;y>=0;y--){
//				  			for(x=0;x<16;x++){
//				  				if((row[y]&(1<<x))==(1<<x)){
//				  					printf("+---");
//				  				}
//				  				else if((row[y]&(0<<x))==(0<<x)){
//				  					printf("+   ");
//				  				}
//				  			}
//				  			printf("+");
//				  			printf("\n");
//				  			printf("|");
//				  		for(xx=0;xx<16;xx++){
//				  			if((column[xx]&(1<<y))==(1<<y)){
//				  				printf("%3d|",Dist_Map[xx][y]);
//				  				}
//				  			else if((column[xx]&(0<<y))==(0<<y)){
//				  				printf("%3d ",Dist_Map[xx][y]);
//				  				}
//				  		}
//				  			printf("\n");
//				  		}
//				  	/*最後の行*/
//				  	for(i=0;i<16;i++){
//				  		printf("+---");
//				  	}
//				  	printf("+\n");
//				  	printf("end\n\n");
//				  }










	     //2周テスト用
//		 trapezoid_accel_lturn(1000,100,400,80,90);//左90°曲がる
//
//						//g_WallControlStatus=g_WallControlStatus|(1<<0);//1bit目を1にする修正済
//
//						}else if(g_sensor[0][0]>THERESHOULD_FL&&g_sensor[3][0]>THERESHOULD_FR){//前壁なし
//							g_WallControlStatus=g_WallControlStatus&~(1<<0);//1bit目を０にする
//
//						}else if((float)g_sensor[2][0]>THERESHOULD_R){
//							g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
//
//						}else{
//							g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
//
//							}
//  	  }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
