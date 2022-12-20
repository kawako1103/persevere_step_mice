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
   int g_V_batt =0;
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

	  // test speaker
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

//		//sensor test
//		 HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
//		 printf("BATT=%f\n",g_V_batt);
//		 printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d\n", g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
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
		 if (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0){
//		 trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
//
//	     //2周テスト用
////		 trapezoid_accel_lturn(1000,100,400,80,90);//左90°曲がる
////		 trapezoid_accel_lturn(1000,100,400,80,90);//90°曲がる
////         trapezoid_accel_lturn(1000,100,400,80,90);//90°曲がる
////         trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////         trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////		 trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////		 trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
////		 trapezoid_accel_rturn(1000,100,400,80,90);//90°曲がる
//
		 trapezoid_accel_rturn(1000,100,400,80,90);//右90°曲がる
		 }

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


////センサー取得
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

//<左手法>の概要
				  float THERESHOULD_L=260;//左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100
				  float THERESHOULD_R=180;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70
				  float THERESHOULD_FL=260;//左壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140
				  float THERESHOULD_FR=180;//右壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220
				  if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
				  if((float)g_sensor[1][0]<THERESHOULD_L){//[1][0]は、左壁, [2][0]は、右壁 //ここは左壁無し
				  //90 mm直進,90°左旋回,90 mm直進,停止
				  trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
		 	 	  trapezoid_accel_lturn(1000,100,400,80,90);//左90°曲がる
		 	 	  trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
				  }else if(g_sensor[0][0]>THERESHOULD_FL&&g_sensor[3][0]>THERESHOULD_FR){//前壁なし
//					g_WallControlStatus=g_WallControlStatus&~(1<<0);//1bit目を０にする
				  //180 mm直進,停止
				  trapezoid_accel_forward(1000,100,500,100,180);//90む台形加速の関数
				  }else if((float)g_sensor[2][0]<THERESHOULD_R){//右壁がない
//					g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
				  //90 mm直進,90°右旋回,90 mm直進,停止
				  trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
				  trapezoid_accel_rturn(1000,100,400,80,90);//左90°曲がる
				  trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
				  }else{
//					g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
				  //90前進,180°左旋回,90前進,停止
				  trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
				  trapezoid_accel_lturn(1000,100,400,80,180);//左90°曲がる
				  trapezoid_accel_forward(1000,100,500,100,90);//90む台形加速の関数
				  }
				  }



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
