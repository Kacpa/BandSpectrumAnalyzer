/*
 * led.h
 *
 *  Created on: Jan 18, 2024
 *      Author: kacpa
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdbool.h>
#include <stdint.h>

extern uint32_t ledbuf[40+256*24];
extern bool ledmatrix[32][8];

void LedInit(uint8_t rowsnb, uint8_t colnb);

void setcolor(int position, uint8_t color_green, uint8_t color_red, uint8_t color_blue);

void updatebuf();

void clearmatrix();

void clearbuf();

void setstripe(int stripe_number, int amp);

#endif /* INC_LED_H_ */
