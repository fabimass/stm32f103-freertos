#ifndef __TFT_DISPLAY_H__
#define __TFT_DISPLAY_H__
#include "main.h"
#include "tft_fonts.h"
#include <stdbool.h>

#define TFT_Display_MADCTL_MY  0x80
#define TFT_Display_MADCTL_MX  0x40
#define TFT_Display_MADCTL_MV  0x20
#define TFT_Display_MADCTL_ML  0x10
#define TFT_Display_MADCTL_RGB 0x00
#define TFT_Display_MADCTL_BGR 0x08
#define TFT_Display_MADCTL_MH  0x04

/*** Redefine if necessary ***/
#define TFT_Display_SPI_PORT hspi1
extern SPI_HandleTypeDef TFT_Display_SPI_PORT;

#define TFT_Display_RES_Pin       	DISPLAY_RESET_Pin
#define TFT_Display_RES_GPIO_Port 	DISPLAY_RESET_GPIO_Port
#define TFT_Display_CS_Pin        	DISPLAY_CS_Pin
#define TFT_Display_CS_GPIO_Port  	DISPLAY_CS_GPIO_Port
#define TFT_Display_DC_Pin        	DISPLAY_DC_Pin
#define TFT_Display_DC_GPIO_Port  	DISPLAY_DC_GPIO_Port
#define TFT_Display_LED_Pin        	DISPLAY_LED_Pin
#define TFT_Display_LED_GPIO_Port  	DISPLAY_LED_GPIO_Port

#define ORIENTED_LEFT

// default orientation
#ifdef ORIENTED_DEFAULT
#define TFT_Display_WIDTH  240
#define TFT_Display_HEIGHT 320
#define TFT_Display_ROTATION (TFT_Display_MADCTL_MX | TFT_Display_MADCTL_BGR)*/
#endif

// rotate right
#ifdef ORIENTED_RIGHT
#define TFT_Display_WIDTH  320
#define TFT_Display_HEIGHT 240
#define TFT_Display_ROTATION (TFT_Display_MADCTL_MX | TFT_Display_MADCTL_MY | TFT_Display_MADCTL_MV | TFT_Display_MADCTL_BGR)
#endif

// rotate left
#ifdef ORIENTED_LEFT
#define TFT_Display_WIDTH  320
#define TFT_Display_HEIGHT 240
#define TFT_Display_ROTATION (TFT_Display_MADCTL_MV | TFT_Display_MADCTL_BGR)
#endif

// upside down
#ifdef ORIENTED_DOWN
#define TFT_Display_WIDTH  240
#define TFT_Display_HEIGHT 320
#define TFT_Display_ROTATION (TFT_Display_MADCTL_MY | TFT_Display_MADCTL_BGR)
#endif

/****************************/

// Color definitions
#define	COLOR_BLACK   	0x0000
#define	COLOR_BLUE    	0x7497
#define	COLOR_RED  	  	0xD0A2
#define	COLOR_GREEN   	0x7646
#define COLOR_PURPLE	 	0xF81F
#define COLOR_YELLOW  	0xFFE0
#define COLOR_WHITE   	0xFFFF
#define COLOR_565(r, g, b) 	(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

/****************************/

#define swap(a,b) 		{int16_t t=a;a=b;b=t;}

void TFT_Display_Select();
void TFT_Display_Unselect();
void TFT_Display_TurnOn();
void TFT_Display_TurnOff();
void TFT_Display_Init(void);
void TFT_Display_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_Display_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void TFT_Display_DrawLine(unsigned int color, unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void TFT_Display_DrawRectangle(unsigned int color,unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void TFT_Display_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void TFT_Display_FillScreen(uint16_t color);
void TFT_Display_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* data);
void TFT_Display_InvertColors(bool invert);

#endif // __TFT_DISPLAY_H__
