#include "lcd.h"

#define LCD_LINE_0 (0)
#define LCD_LINE_1 (22)
#define LCD_LINE_2 (44)

void lcdInit(I2C_HandleTypeDef * i2ch)
{
  ssd1306_Init(i2ch);
  ssd1306_SetCursor(0, LCD_LINE_0);
  char buf[50] = {0};
  sprintf(buf, "OrionSub V4");
  ssd1306_WriteString(buf, Font_11x18, White);

  ssd1306_SetCursor(0, LCD_LINE_1);
  sprintf(buf, "%s", __DATE__);
  ssd1306_WriteString(buf, Font_11x18, White);

  ssd1306_SetCursor(0, LCD_LINE_2);
  sprintf(buf, "%s", __TIME__);
  ssd1306_WriteString(buf, Font_11x18, White);

  ssd1306_UpdateScreen(i2ch);
}

void lcdPrint(I2C_HandleTypeDef * i2ch, float batt_v, int can_rx, int ball_0, int ball_1)
{
  ssd1306_Fill(Black);
  char buf[50] = {0};

  ssd1306_SetCursor(0, LCD_LINE_0);
  sprintf(buf, "%3.1fV %3d", batt_v, can_rx);
  ssd1306_WriteString(buf, Font_11x18, White);

  ssd1306_SetCursor(0, LCD_LINE_1);
  sprintf(buf, "%+4d %+4d", ball_0, ball_1);
  ssd1306_WriteString(buf, Font_11x18, White);

  ssd1306_SetCursor(0, LCD_LINE_2);
  sprintf(buf, "push4update");
  ssd1306_WriteString(buf, Font_11x18, White);

  ssd1306_UpdateScreen(i2ch);
}