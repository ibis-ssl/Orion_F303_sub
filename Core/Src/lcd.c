#include "lcd.h"

void lcdInit(I2C_HandleTypeDef * i2ch)
{
  ssd1306_Init(i2ch);
  ssd1306_SetCursor(0, 0);
  char str[] = "orion-sub v4";
  FontDef font;
  ssd1306_WriteString(str, font, Black);
}