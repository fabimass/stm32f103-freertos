#include "stm32f1xx_hal.h"
#include "tft_display.h"
#include <stdlib.h>

void TFT_Display_Select() {
    HAL_GPIO_WritePin(TFT_Display_CS_GPIO_Port, TFT_Display_CS_Pin, GPIO_PIN_RESET);
}

void TFT_Display_Unselect() {
    HAL_GPIO_WritePin(TFT_Display_CS_GPIO_Port, TFT_Display_CS_Pin, GPIO_PIN_SET);
}

void TFT_Display_TurnOn() {
    HAL_GPIO_WritePin(TFT_Display_LED_GPIO_Port, TFT_Display_LED_Pin, GPIO_PIN_SET);
}

void TFT_Display_TurnOff() {
    HAL_GPIO_WritePin(TFT_Display_LED_GPIO_Port, TFT_Display_LED_Pin, GPIO_PIN_RESET);
}

static void TFT_Display_Reset() {
    HAL_GPIO_WritePin(TFT_Display_RES_GPIO_Port, TFT_Display_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(TFT_Display_RES_GPIO_Port, TFT_Display_RES_Pin, GPIO_PIN_SET);
}

static void TFT_Display_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(TFT_Display_DC_GPIO_Port, TFT_Display_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&TFT_Display_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

static void TFT_Display_WriteData(uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(TFT_Display_DC_GPIO_Port, TFT_Display_DC_Pin, GPIO_PIN_SET);

    // split data in small chunks because HAL can't send more then 64K at once
    while(buff_size > 0) {
        uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
        HAL_SPI_Transmit(&TFT_Display_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
        buff += chunk_size;
        buff_size -= chunk_size;
    }
}

static void TFT_Display_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // column address set
	TFT_Display_WriteCommand(0x2A); // CASET
    {
        uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // row address set
    TFT_Display_WriteCommand(0x2B); // RASET
    {
        uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // write to RAM
    TFT_Display_WriteCommand(0x2C); // RAMWR
}

void TFT_Display_Init() {
	TFT_Display_Select();
	TFT_Display_Reset();

    // command list is based on https://github.com/martnak/STM32-ILI9341

    // SOFTWARE RESET
	TFT_Display_WriteCommand(0x01);
    HAL_Delay(1000);
        
    // POWER CONTROL A
    TFT_Display_WriteCommand(0xCB);
    {
        uint8_t data[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // POWER CONTROL B
    TFT_Display_WriteCommand(0xCF);
    {
        uint8_t data[] = { 0x00, 0xC1, 0x30 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL A
    TFT_Display_WriteCommand(0xE8);
    {
        uint8_t data[] = { 0x85, 0x00, 0x78 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL B
    TFT_Display_WriteCommand(0xEA);
    {
        uint8_t data[] = { 0x00, 0x00 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // POWER ON SEQUENCE CONTROL
    TFT_Display_WriteCommand(0xED);
    {
        uint8_t data[] = { 0x64, 0x03, 0x12, 0x81 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // PUMP RATIO CONTROL
    TFT_Display_WriteCommand(0xF7);
    {
        uint8_t data[] = { 0x20 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,VRH[5:0]
    TFT_Display_WriteCommand(0xC0);
    {
        uint8_t data[] = { 0x23 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,SAP[2:0];BT[3:0]
    TFT_Display_WriteCommand(0xC1);
    {
        uint8_t data[] = { 0x10 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // VCM CONTROL
    TFT_Display_WriteCommand(0xC5);
    {
        uint8_t data[] = { 0x3E, 0x28 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // VCM CONTROL 2
    TFT_Display_WriteCommand(0xC7);
    {
        uint8_t data[] = { 0x86 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // MEMORY ACCESS CONTROL
    TFT_Display_WriteCommand(0x36);
    {
        uint8_t data[] = { 0x48 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // PIXEL FORMAT
    TFT_Display_WriteCommand(0x3A);
    {
        uint8_t data[] = { 0x55 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // FRAME RATIO CONTROL, STANDARD RGB COLOR
    TFT_Display_WriteCommand(0xB1);
    {
        uint8_t data[] = { 0x00, 0x18 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // DISPLAY FUNCTION CONTROL
    TFT_Display_WriteCommand(0xB6);
    {
        uint8_t data[] = { 0x08, 0x82, 0x27 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // 3GAMMA FUNCTION DISABLE
    TFT_Display_WriteCommand(0xF2);
    {
        uint8_t data[] = { 0x00 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // GAMMA CURVE SELECTED
    TFT_Display_WriteCommand(0x26);
    {
        uint8_t data[] = { 0x01 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // POSITIVE GAMMA CORRECTION
    TFT_Display_WriteCommand(0xE0);
    {
        uint8_t data[] = { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                           0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // NEGATIVE GAMMA CORRECTION
    TFT_Display_WriteCommand(0xE1);
    {
        uint8_t data[] = { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                           0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F };
        TFT_Display_WriteData(data, sizeof(data));
    }

    // EXIT SLEEP
    TFT_Display_WriteCommand(0x11);
    HAL_Delay(120);

    // TURN ON DISPLAY
    TFT_Display_WriteCommand(0x29);

    // MADCTL
    TFT_Display_WriteCommand(0x36);
    {
        uint8_t data[] = { TFT_Display_ROTATION };
        TFT_Display_WriteData(data, sizeof(data));
    }

    TFT_Display_TurnOn();

    TFT_Display_Unselect();
}

void TFT_Display_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= TFT_Display_WIDTH) || (y >= TFT_Display_HEIGHT))
        return;

    TFT_Display_Select();

    TFT_Display_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    TFT_Display_WriteData(data, sizeof(data));

    TFT_Display_Unselect();
}

static void TFT_Display_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    TFT_Display_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                TFT_Display_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                TFT_Display_WriteData(data, sizeof(data));
            }
        }
    }
}

void TFT_Display_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
	TFT_Display_Select();

    while(*str) {
        if(x + font.width >= TFT_Display_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= TFT_Display_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        TFT_Display_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    TFT_Display_Unselect();
}

void TFT_Display_DrawLine(unsigned int color, unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
	int steep = abs(y2-y1) > abs(x2-x1);

	if (steep)
	{
		swap(x1,y1);
		swap(x2,y2);
	}

	if(x1>x2)
	{
		swap(x1,x2);
		swap(y1,y2);
	}

	int dx,dy;
	dx = (x2 - x1);
	dy = abs(y2 - y1);
	int err = dx / 2;
	int ystep;
	if(y1 < y2)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}
	for (; x1 <= x2; x1++)
	{
		if (steep)
		{
			TFT_Display_DrawPixel(y1, x1, color);

		}
		else
		{
			TFT_Display_DrawPixel(x1, y1, color);
		}
		err -= dy;
		if (err < 0)
		{
			y1 += ystep;
			err = dx;
		}
	}
}

void TFT_Display_DrawRectangle(unsigned int color,unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
	TFT_Display_DrawLine(color, x1, y1, x2, y1);
	TFT_Display_DrawLine(color, x2, y1, x2, y2);
	TFT_Display_DrawLine(color, x1, y1, x1, y2);
	TFT_Display_DrawLine(color, x1, y2, x2, y2);
}

void TFT_Display_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= TFT_Display_WIDTH) || (y >= TFT_Display_HEIGHT)) return;
    if((x + w - 1) >= TFT_Display_WIDTH) w = TFT_Display_WIDTH - x;
    if((y + h - 1) >= TFT_Display_HEIGHT) h = TFT_Display_HEIGHT - y;

    TFT_Display_Select();
    TFT_Display_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    HAL_GPIO_WritePin(TFT_Display_DC_GPIO_Port, TFT_Display_DC_Pin, GPIO_PIN_SET);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            HAL_SPI_Transmit(&TFT_Display_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }

    TFT_Display_Unselect();
}

void TFT_Display_FillScreen(uint16_t color) {
	TFT_Display_FillRectangle(0, 0, TFT_Display_WIDTH, TFT_Display_HEIGHT, color);
}

void TFT_Display_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* data) {
    if((x >= TFT_Display_WIDTH) || (y >= TFT_Display_HEIGHT)) return;
    if((x + w - 1) >= TFT_Display_WIDTH) return;
    if((y + h - 1) >= TFT_Display_HEIGHT) return;

    TFT_Display_Select();
    TFT_Display_SetAddressWindow(x, y, x+w-1, y+h-1);
    TFT_Display_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    TFT_Display_Unselect();
}

void TFT_Display_InvertColors(bool invert) {
	TFT_Display_Select();
	TFT_Display_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
	TFT_Display_Unselect();
}

