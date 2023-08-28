#include "stm32f1xx_hal.h"
#include "tft_touch.h"

#define READ_X 0xD0
#define READ_Y 0x90
#define NSAMPLES 16

void TFT_Touch_Select() {
    HAL_GPIO_WritePin(TFT_TOUCH_CS_GPIO_Port, TFT_TOUCH_CS_Pin, GPIO_PIN_RESET);
}

void TFT_Touch_Unselect() {
    HAL_GPIO_WritePin(TFT_TOUCH_CS_GPIO_Port, TFT_TOUCH_CS_Pin, GPIO_PIN_SET);
}

bool TFT_Touch_ScreenPressed() {
    return HAL_GPIO_ReadPin(TFT_TOUCH_IRQ_GPIO_Port, TFT_TOUCH_IRQ_Pin) == GPIO_PIN_RESET;
}

bool TFT_Touch_GetCoordinates(uint16_t* x, uint16_t* y) {
    static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };

    TFT_Touch_Select();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;
    uint8_t x_raw[2], y_raw[2];

    for(uint8_t i = 0; i < NSAMPLES; i++) {
        if(!TFT_Touch_ScreenPressed())
            break;

        nsamples++;

        /* Read Y coords: Have to transmit the command and then read the next 2 bytes */
        HAL_SPI_Transmit(&TFT_TOUCH_SPI_PORT, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        HAL_SPI_TransmitReceive(&TFT_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);

        /* Read X coords: Have to transmit the command and then read the next 2 bytes */
        HAL_SPI_Transmit(&TFT_TOUCH_SPI_PORT, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        HAL_SPI_TransmitReceive(&TFT_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);

        /* The values are conformed by 12 bits only */
        avg_x += (((uint16_t)x_raw[0]) << 4) | ((uint16_t)x_raw[1] >> 4);
        avg_y += (((uint16_t)y_raw[0]) << 4) | ((uint16_t)y_raw[1] >> 4);
    }

    TFT_Touch_Unselect();

    if(nsamples < NSAMPLES)
        return false;

    avg_x = (avg_x / NSAMPLES);
    if(avg_x < TFT_TOUCH_MIN_RAW_X) avg_x = TFT_TOUCH_MIN_RAW_X;
    if(avg_x > TFT_TOUCH_MAX_RAW_X) avg_x = TFT_TOUCH_MAX_RAW_X;

    avg_y = (avg_y / 16);
    if(avg_y < TFT_TOUCH_MIN_RAW_X) avg_y = TFT_TOUCH_MIN_RAW_Y;
    if(avg_y > TFT_TOUCH_MAX_RAW_Y) avg_y = TFT_TOUCH_MAX_RAW_Y;

    *x = (avg_x - TFT_TOUCH_MIN_RAW_X) * TFT_TOUCH_SCALE_X / (TFT_TOUCH_MAX_RAW_X - TFT_TOUCH_MIN_RAW_X);
    *y = (avg_y - TFT_TOUCH_MIN_RAW_Y) * TFT_TOUCH_SCALE_Y / (TFT_TOUCH_MAX_RAW_Y - TFT_TOUCH_MIN_RAW_Y);

    return true;
}
