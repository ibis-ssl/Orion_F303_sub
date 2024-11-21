#ifndef LCD_DISP_H_
#define LCD_DISP_H_

#include "main.h"
#include "ssd1306.h"
#include "stm32f3xx_hal_i2c.h"

// lib
#include "fonts.h"
#include "ssd1306.h"

void lcdInit(I2C_HandleTypeDef * i2ch);

#endif