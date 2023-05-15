/*
 * tftspi_st7735.h
 *
 *  Created on: 3 thg 6, 2022
 *      Author: A315-56
 */

#ifndef TFTSPI_ST7735_H_
#define TFTSPI_ST7735_H_

#pragma once
#include "component_config.h"
#if ENABLE_COMPONENT_TFTSPI

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "stdio.h"
#include "stdbool.h"
#include "stm32f1xx.h"
#include "tftspi_st7735/fonts.h"
#include "tftspi_st7735/tftspi_settings.h"

#include "periph/spi.h"

//#define TFT_CSPIN_EN
//#define USE_EX_FLASH
#ifdef USE_EX_FLASH
#include "SPIFLASH.h"
#endif

#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFDA0      /* 255, 180,   0 */
#define GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define PINK        0xFC9F

extern volatile uint8_t dma_tft_flag;
extern volatile int8_t dma_tft_cnt;

typedef enum{
	TFT_COLOR_RGB = ST7735_MADCTL_RGB,
	TFT_COLOR_BGR = ST7735_MADCTL_BGR
} tftspi_colorformat_t;

class TFTSPI_ST7735{
	public:
		const uint16_t COLOR[19] = {BLACK, NAVY, DARKGREEN, DARKCYAN, MAROON, PURPLE, OLIVE, LIGHTGREY, DARKGREY, BLUE, GREEN, CYAN, RED, MAGENTA, YELLOW, WHITE, ORANGE, GREENYELLOW, PINK};
#ifdef TFT_CSPIN_EN
		TFTSPI_ST7735(GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *rst_port, uint16_t rst_pin, GPIO_TypeDef *dc_port, uint16_t dc_pin);
#endif
		TFTSPI_ST7735(GPIO_TypeDef *rst_port, uint16_t rst_pin, GPIO_TypeDef *dc_port, uint16_t dc_pin);

		void init(spi_t spi, tftspi_colorformat_t format);

		void set_rotation(uint8_t m);
		void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
		void write_color(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *Data, uint16_t size);

		void draw_pixel(uint16_t x, uint16_t y, uint16_t color);
		void draw_line(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t color);
		void draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

		void draw_rgb_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* Bitmap);
		void draw_MkE_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t Color, uint16_t B_Color, const uint8_t* Bitmap);

		uint8_t draw_char(uint16_t x, uint16_t y, Font Font, uint16_t Color, uint16_t B_Color, char *Char);
		void print(uint16_t x, uint16_t y, Font Font, uint16_t Color, uint16_t B_Color, char *str);

		void fill_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
		void fill_screen(uint16_t color);

		uint16_t RGB565(uint8_t R, uint8_t G, uint8_t B);

#ifdef USE_EX_FLASH
		void WriteRGBBitmapToFlash(SPIFLASH Flash, uint32_t Write_Address, uint16_t w, uint16_t h, const uint16_t *Bitmap);
		void DrawRGBBitmapInFlash(SPIFLASH Flash, uint32_t Read_Address, uint16_t x, uint16_t y);

#endif
	private:
		GPIO_TypeDef *rst_port, *dc_port;
		uint16_t rst_pin, dc_pin;
		uint8_t X_Start, Y_Start;
		tftspi_colorformat_t _format = TFT_COLOR_RGB;
		spi_t _spi;
#ifdef TFT_CSPIN_EN
		GPIO_TypeDef *cs_port;
		uint16_t cs_pin;
		void Active(void);
		void Idle(void);
#endif
		void EnableWrite(void);
		void Reset(void);
		void WriteCommand(uint8_t cmd);
		void WriteData(uint8_t* buff, size_t buff_size);
		void InitCommandList(const uint8_t *addr);
		uint8_t UTF8_GetAddr(char *utf8_char, uint8_t *char_offset);
};


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_TFTSPI */

#endif /* TFTSPI_ST7735_H_ */
