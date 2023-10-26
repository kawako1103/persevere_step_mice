/*
 * calPWMvel.c
 *
 *  Created on: Dec 13, 2022
 *      Author: kawaguchitakahito
 */
#include "adc.h"
#include "dma.h"
#include "gpio.h"//
#include <math.h>
//float TIREDIAMETER = 51.86*2694/2700*2700/2745; //51.86 mmのタイヤ径 距離の調整により52
///
///
///一歩一歩だと↓
//float TIREDIAMETER = 51.86*2655/2700*2670/2700*2750/2700*2664/2700;//*2725/2700*2666/2700; //51.86 mmのタイヤ径 距離の調整により52

///
///
//連続走行の場合(最初と最後以外, 加速、減速区間がないver.は↓)
float TIREDIAMETER = 51.86*2655/2700*2670/2700*2750/2700*2664/2700*2751.5/2700*2687/2700*2680/2700;//*2725/2700*2666/2700; //51.86 mmのタイヤ径 距離の調整により52


uint16_t calPWMCount(float velocity) {

        uint16_t PWMCount;

        if ((fabs(velocity) > 0.0)
                        && (78539.8163 / fabs(velocity)*TIREDIAMETER < UINT16_MAX)) {
        	//カウンタクロック65535→10000に変更//ミスあり　10000→10MHzに変更　0.9×π/180×1/2×10^7
                PWMCount = (uint16_t) (78539.8163
                      / fabs(velocity) * TIREDIAMETER) - 1;
        } else {
                PWMCount = UINT16_MAX - 1;
        }

        return PWMCount;
}
