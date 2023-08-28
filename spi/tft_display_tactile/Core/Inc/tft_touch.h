#ifndef __TFT_TOUCH_H__
#define __TFT_TOUCH_H__
#include "main.h"
#include <stdbool.h>

/*** Redefine if necessary ***/

#define TFT_TOUCH_SPI_PORT hspi1
extern SPI_HandleTypeDef TFT_TOUCH_SPI_PORT;

#define TFT_TOUCH_IRQ_Pin       TOUCH_IRQ_Pin
#define TFT_TOUCH_IRQ_GPIO_Port TOUCH_IRQ_GPIO_Port
#define TFT_TOUCH_CS_Pin        TOUCH_CS_Pin
#define TFT_TOUCH_CS_GPIO_Port  TOUCH_CS_GPIO_Port

// change depending on screen orientation
#define TFT_TOUCH_SCALE_X 240
#define TFT_TOUCH_SCALE_Y 320

// change after calibration
#define TFT_TOUCH_MIN_RAW_X 250
#define TFT_TOUCH_MAX_RAW_X 1800
#define TFT_TOUCH_MIN_RAW_Y 250
#define TFT_TOUCH_MAX_RAW_Y 1800

void TFT_Touch_Select();
void TFT_Touch_Unselect();
bool TFT_Touch_ScreenPressed();
bool TFT_Touch_GetCoordinates(uint16_t* x, uint16_t* y);

typedef struct{
	uint16_t x;
	uint16_t y;
}TFT_TOUCH_COORDS;

#endif // __TFT_TOUCH_H__
