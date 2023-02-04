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
float TIREDIAMETER = 51.86; //
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
