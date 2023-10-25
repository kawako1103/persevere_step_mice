/*
 * PL_sensor.c
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */


#include <PL_sensor.h>
#include "adc.h"
#include "dma.h"
#include "gpio.h"//sensorLED用


uint16_t g_ADCBuffer[5];
char AD_step;

uint16_t g_sensor_on[4];
uint16_t g_sensor_off[4];
uint16_t g_sensor[4][4];

float g_V_batt;


/*******************************************************************/
/*	電圧の取得			(pl_getbatt)	*/
/*******************************************************************/
/*	戻り値に電圧を返す．(AD変換単体)						*/
/*******************************************************************/
float pl_getbatt(void){
	 float batt;
	 uint16_t battAD;

	//HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);一旦外す
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	battAD = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	batt = 3.3  * (float) battAD / 1023.0 * (100.0 + 22.0) / 22.0;


return batt;
}

/*******************************************************************/
/*	callback用関数			(pl_callback_getSensor)	*/
/*******************************************************************/
/*	DMAがスタートしたら実行するコード(AD変換複数)					*/
/*******************************************************************/
void pl_callback_getSensor(void) {//センサー取得概要の右のCallbackのところ、(g_sensor_offにg_ADCBufferの結果を入れるとか)ちょこちょこ異なるがこれでひとまずよさそう
uint16_t V_battAD;

int j;
	HAL_ADC_Stop_DMA(&hadc1);
//	printf("%4d,%4d,%4d,%4d\n", g_ADCBuffer[1], g_ADCBuffer[2], g_ADCBuffer[3],g_ADCBuffer[4]);
//HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));


//}
//HAL_ADC_Stop_DMA(&hadc1);
switch (AD_step) {
case 0:
HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_RESET);
for (j = 0; j <= 500; j++) {
}
break;
case 1:
g_sensor_on[0] = g_ADCBuffer[1];
g_sensor_on[1] = g_ADCBuffer[2];
g_sensor_off[2] = g_ADCBuffer[3];
g_sensor_off[3] = g_ADCBuffer[4];
HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_SET);
for (j = 0; j <= 500; j++) {
}
break;
case 2:
g_sensor_off[0] = g_ADCBuffer[1];
g_sensor_off[1] = g_ADCBuffer[2];
g_sensor_on[2] = g_ADCBuffer[3];
g_sensor_on[3] = g_ADCBuffer[4];
HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_RESET);
for (j = 0; j <= 10; j++) {
}
break;
}
V_battAD = g_ADCBuffer[0];
g_V_batt = 3.3 * (float) V_battAD / 1023 * (100.0 + 47.0) / 47.0;
AD_step++;

if (AD_step != 3) {
HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
sizeof(g_ADCBuffer) / sizeof(uint16_t));

} else {
AD_step = 0;

//if (sensor_mode=1)(
for(j=3;j<=1;j--){
	g_sensor[0][j]=g_sensor[0][j-1];
	g_sensor[1][j]=g_sensor[1][j-1];
	g_sensor[2][j]=g_sensor[2][j-1];
	g_sensor[3][j]=g_sensor[3][j-1];
}
g_sensor[0][0]=g_sensor_on[0]-g_sensor_off[0];
g_sensor[1][0]=g_sensor_on[1]-g_sensor_off[1];
g_sensor[2][0]=g_sensor_on[2]-g_sensor_off[2];
g_sensor[3][0]=g_sensor_on[3]-g_sensor_off[3];
}

}





/*******************************************************************/
/*	割り込み用動作関数(センサー取得)			(interupt_calSensor)	*/
/*******************************************************************/
/*	センサーの情報を取得する割り込み関数．						*/
/*******************************************************************/
void pl_interupt_getSensor(void){
HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));


}
