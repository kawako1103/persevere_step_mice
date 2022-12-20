/*
 * PL_lcd.h
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */

#ifndef INC_PL_LCD_H_
#define INC_PL_LCD_H_

#include "stm32l4xx_hal.h"


#define LCD_ADDRESS 0x7c

void lcd_cmd(uint8_t);
void lcd_data(uint8_t);

void pl_lcd_puts(const char *);

void pl_lcd_init();

void pl_lcd_move(uint8_t);
void pl_lcd_pos(uint8_t, uint8_t);

void pl_lcd_clear();

#endif /* INC_PL_LCD_H_ */
