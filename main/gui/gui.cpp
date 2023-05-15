/*
 * gui.cpp
 *
 *  Created on: May 14, 2023
 *      Author: anh
 */

#include "gui/gui.h"
#include "stdlib.h"
#include "string.h"



extern TFTSPI_ST7735 tft;
extern volatile int16_t Encoder_Cnt;
extern uint8_t ENC_MENU_PREV1;
extern uint8_t ENC_MENU_PREV2;


const uint8_t Ubuntu36x50[12][252] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xFC, 0xFF, 0x03, 0x00, 0xFE, 0xFF, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xFC, 0x0F, 0x00, 0x00, 0xC0, 0xFF, 0x00, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFC, 0x07, 0x00, 0x00, 0x80, 0xFF, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xFC, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0x00, 0x00},  // Code for char 0
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 1
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xE0, 0x07, 0x00, 0x00, 0xE0, 0xFF, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xFE, 0xFF, 0x00, 0xF8, 0x0F, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xF8, 0x0F, 0x00, 0xC0, 0xFF, 0xFF, 0x00, 0xFC, 0x07, 0x00, 0xE0, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0xE0, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0xF0, 0xFF, 0xFF, 0x00, 0xFE, 0x03, 0x00, 0xF8, 0xFF, 0xFF, 0x00, 0xFE, 0x01, 0x00, 0xFC, 0xFF, 0xFF, 0x00, 0xFE, 0x01, 0x00, 0xFE, 0x7F, 0xFF, 0x00, 0xFE, 0x01, 0x00, 0xFF, 0x3F, 0xFF, 0x00, 0xFE, 0x01, 0x80, 0xFF, 0x1F, 0xFF, 0x00, 0xFE, 0x01, 0xC0, 0xFF, 0x07, 0xFF, 0x00, 0xFE, 0x01, 0xE0, 0xFF, 0x03, 0xFF, 0x00, 0xFE, 0x03, 0xF0, 0xFF, 0x01, 0xFF, 0x00, 0xFE, 0x03, 0xF8, 0xFF, 0x00, 0xFF, 0x00, 0xFE, 0x0F, 0xFE, 0x7F, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x3F, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x1F, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x0F, 0x00, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0x07, 0x00, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0x00, 0xF0, 0xFF, 0xFF, 0x01, 0x00, 0xFF, 0x00, 0xE0, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xC0, 0xFF, 0x3F, 0x00, 0x00, 0xFF, 0x00, 0x80, 0xFF, 0x0F, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 2
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x80, 0x7F, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x80, 0xFF, 0x00, 0xF8, 0x0F, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xF8, 0x07, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFC, 0x07, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFC, 0x03, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFC, 0x03, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF8, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x03, 0xF8, 0x1F, 0x00, 0xFF, 0x01, 0xFE, 0x07, 0xFC, 0x1F, 0x00, 0xFF, 0x01, 0xFE, 0x0F, 0xFE, 0x3F, 0x80, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x7F, 0x80, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xE0, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0x7F, 0xFF, 0xFF, 0x7F, 0x00, 0xF0, 0xFF, 0x7F, 0xFF, 0xFF, 0x3F, 0x00, 0xE0, 0xFF, 0x3F, 0xFE, 0xFF, 0x3F, 0x00, 0xC0, 0xFF, 0x1F, 0xFE, 0xFF, 0x1F, 0x00, 0x00, 0xFF, 0x07, 0xFC, 0xFF, 0x0F, 0x00, 0x00, 0xFC, 0x01, 0xF8, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 3
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xCF, 0x3F, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xC3, 0x3F, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xC1, 0x3F, 0x00, 0x00, 0x00, 0xFC, 0x7F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0xFE, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x80, 0xFF, 0x0F, 0xC0, 0x3F, 0x00, 0x00, 0xC0, 0xFF, 0x07, 0xC0, 0x3F, 0x00, 0x00, 0xE0, 0xFF, 0x03, 0xC0, 0x3F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0xF8, 0x7F, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00},  // Code for char 4
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F, 0x00, 0x00, 0x00, 0xC0, 0x07, 0x00, 0xFF, 0x00, 0x00, 0xFC, 0xFF, 0x07, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0xFE, 0x01, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0xFE, 0x01, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0xFE, 0x01, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0xFE, 0x01, 0xFC, 0xFF, 0xFF, 0x0F, 0x00, 0xFE, 0x01, 0xFC, 0xFF, 0xFB, 0x0F, 0x00, 0xFE, 0x01, 0xFC, 0x03, 0xF8, 0x0F, 0x00, 0xFE, 0x01, 0xFC, 0x03, 0xF8, 0x0F, 0x00, 0xFE, 0x01, 0xFC, 0x03, 0xF8, 0x0F, 0x00, 0xFE, 0x01, 0xFC, 0x03, 0xF8, 0x1F, 0x00, 0xFF, 0x01, 0xFC, 0x03, 0xF0, 0x1F, 0x00, 0xFF, 0x01, 0xFC, 0x03, 0xF0, 0x3F, 0x00, 0xFF, 0x00, 0xFC, 0x03, 0xF0, 0x7F, 0x80, 0xFF, 0x00, 0xFC, 0x03, 0xF0, 0xFF, 0xE0, 0xFF, 0x00, 0xFC, 0x03, 0xE0, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0xE0, 0xFF, 0xFF, 0x7F, 0x00, 0xFC, 0x03, 0xC0, 0xFF, 0xFF, 0x7F, 0x00, 0xFC, 0x03, 0xC0, 0xFF, 0xFF, 0x3F, 0x00, 0xFC, 0x03, 0x80, 0xFF, 0xFF, 0x3F, 0x00, 0xFC, 0x03, 0x00, 0xFF, 0xFF, 0x1F, 0x00, 0xFC, 0x03, 0x00, 0xFE, 0xFF, 0x0F, 0x00, 0xFC, 0x03, 0x00, 0xFC, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 5
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x80, 0xFF, 0xFF, 0x0F, 0xF8, 0xFF, 0x00, 0x80, 0xFF, 0xF7, 0x0F, 0xC0, 0xFF, 0x00, 0xC0, 0xFF, 0xF3, 0x07, 0x80, 0xFF, 0x00, 0xC0, 0xFF, 0xF8, 0x07, 0x00, 0xFF, 0x01, 0xE0, 0x7F, 0xF8, 0x07, 0x00, 0xFF, 0x01, 0xE0, 0x3F, 0xF8, 0x07, 0x00, 0xFE, 0x01, 0xF0, 0x3F, 0xF8, 0x07, 0x00, 0xFE, 0x01, 0xF0, 0x1F, 0xF8, 0x07, 0x00, 0xFE, 0x01, 0xF0, 0x0F, 0xF8, 0x07, 0x00, 0xFE, 0x01, 0xF8, 0x0F, 0xF8, 0x07, 0x00, 0xFE, 0x01, 0xF8, 0x0F, 0xF8, 0x0F, 0x00, 0xFF, 0x01, 0xF8, 0x07, 0xF8, 0x0F, 0x00, 0xFF, 0x01, 0xF8, 0x07, 0xF8, 0x1F, 0xC0, 0xFF, 0x00, 0xF8, 0x07, 0xF0, 0x7F, 0xE0, 0xFF, 0x00, 0xFC, 0x07, 0xF0, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0xF0, 0xFF, 0xFF, 0x7F, 0x00, 0xFC, 0x03, 0xE0, 0xFF, 0xFF, 0x7F, 0x00, 0xFC, 0x03, 0xE0, 0xFF, 0xFF, 0x3F, 0x00, 0xFC, 0x03, 0xC0, 0xFF, 0xFF, 0x1F, 0x00, 0xFC, 0x03, 0x80, 0xFF, 0xFF, 0x0F, 0x00, 0xFC, 0x03, 0x00, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1F, 0x00, 0x00},  // Code for char 6
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0xF0, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x80, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0x00, 0xF8, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0xE0, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0xFC, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0xE0, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0xF8, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x03, 0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x83, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0xFC, 0xC3, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0xFC, 0xF3, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0xFC, 0xFB, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 7
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0xF8, 0x03, 0xC0, 0xFF, 0x03, 0x00, 0x00, 0xFF, 0x0F, 0xF0, 0xFF, 0x07, 0x00, 0x80, 0xFF, 0x1F, 0xF8, 0xFF, 0x1F, 0x00, 0xC0, 0xFF, 0x7F, 0xFC, 0xFF, 0x1F, 0x00, 0xE0, 0xFF, 0xFF, 0xFE, 0xFF, 0x3F, 0x00, 0xF0, 0xFF, 0xFF, 0xFE, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0x00, 0xFC, 0x0F, 0xFF, 0x3F, 0x80, 0xFF, 0x01, 0xFE, 0x03, 0xFC, 0x1F, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0xF8, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF8, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x0F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x1F, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0xF0, 0x1F, 0x00, 0xFE, 0x01, 0xFE, 0x03, 0xF0, 0x3F, 0x00, 0xFE, 0x01, 0xFE, 0x03, 0xF8, 0x3F, 0x00, 0xFF, 0x01, 0xFE, 0x0F, 0xFE, 0x7F, 0x80, 0xFF, 0x01, 0xFC, 0xFF, 0xFF, 0xFF, 0xC1, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0x7F, 0xFF, 0xFF, 0x7F, 0x00, 0xF0, 0xFF, 0x3F, 0xFE, 0xFF, 0x3F, 0x00, 0xE0, 0xFF, 0x1F, 0xFE, 0xFF, 0x1F, 0x00, 0xC0, 0xFF, 0x0F, 0xFC, 0xFF, 0x1F, 0x00, 0x00, 0xFF, 0x07, 0xF8, 0xFF, 0x07, 0x00, 0x00, 0xFC, 0x01, 0xE0, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 8
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0xFF, 0x0F, 0x00, 0xFF, 0x00, 0xF0, 0xFF, 0xFF, 0x0F, 0x00, 0xFF, 0x00, 0xF0, 0xFF, 0xFF, 0x1F, 0x00, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0x1F, 0x00, 0xFF, 0x00, 0xF8, 0xFF, 0xFF, 0x3F, 0x00, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0x3F, 0x00, 0xFF, 0x00, 0xFC, 0x1F, 0xF8, 0x3F, 0x80, 0xFF, 0x00, 0xFE, 0x07, 0xE0, 0x7F, 0x80, 0x7F, 0x00, 0xFE, 0x03, 0xC0, 0x7F, 0x80, 0x7F, 0x00, 0xFE, 0x03, 0xC0, 0x7F, 0x80, 0x7F, 0x00, 0xFE, 0x01, 0x80, 0x7F, 0xC0, 0x7F, 0x00, 0xFE, 0x01, 0x80, 0x7F, 0xC0, 0x7F, 0x00, 0xFE, 0x01, 0x80, 0x7F, 0xE0, 0x3F, 0x00, 0xFE, 0x01, 0x80, 0x7F, 0xE0, 0x3F, 0x00, 0xFE, 0x01, 0x80, 0x7F, 0xF0, 0x3F, 0x00, 0xFE, 0x03, 0x80, 0x7F, 0xF8, 0x1F, 0x00, 0xFE, 0x07, 0x80, 0x3F, 0xFE, 0x1F, 0x00, 0xFC, 0x0F, 0xC0, 0xBF, 0xFF, 0x0F, 0x00, 0xFC, 0x7F, 0xC0, 0xFF, 0xFF, 0x0F, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Code for char 9
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xFC, 0xFF, 0x03, 0x00, 0xFE, 0xFF, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xFC, 0x0F, 0x00, 0x00, 0xC0, 0xFF, 0x00, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x01, 0xFC, 0x07, 0x00, 0x00, 0x80, 0xFF, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0xFC, 0xFF, 0x00, 0xF8, 0x7F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0xF8, 0x7F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0xF0, 0x7F, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0xE0, 0x7F, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0xC0, 0x7F, 0x00, 0x00, 0xFC, 0x0F, 0x00, 0x80, 0x7F, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x00, 0x7E, 0x00, 0x00, 0xFC, 0x01, 0x00, 0x00, 0x78, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // Code for char :
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // Code for char F
};


const uint8_t Degree_Ubuntu16x50[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
		0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xE0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xC0, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0E, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00,};

const char *Header_Screen_Item[] = {"      CÀI ĐẶT       ",
									"   ĐANG GIA NHIỆT   ",
									"     CHẾ ĐỘ NGỦ     ",
									"  CÀI ĐẶT NHIỆT ĐỘ  ",
									"  CÀI ĐẶT HỆ THỐNG  ",
									" CÀI ĐẶT CHẾ ĐỘ NGỦ ",
									" HIỆU CHỈNH NHIỆT ĐỘ ",
									"  CÀI ĐẶT THÔNG SỐ  ",
									"   HIỆU CHỈNH PID   ",
									"  CÀI ĐẶT NGÀY GIỜ  ",
									"     THÔNG TIN      "
};
const char *Menu_Scr_Item[]	     = {"Đặt nhiệt độ hàn.",
									"Cài đặt hệ thống.",
									"Chế độ ngủ.      ",
									"Hiệu chỉnh nhiệt.",
									"Cài đặt thông số.",
									"Hiệu chỉnh PID.  ",
									"Cài đặt ngày giờ.",
									"Thông tin.       ",
									"Thoát.           "};

const char *System_Scr_Item[]	 = {"Đơn vị nhiệt độ. ",
									"Loại điều khiển. ",
									"Bật/tắt buzzer.  ",
									"Chọn chủ đề.     ",
									"Thoát.           "};
const char *Sleep_Scr_Item[]     = {"Nhiệt độ khi ngủ.",
									"Thời gian chờ ngủ",
									"Cho phép thức dậy",
									"Thoát.           "};
const char *Calib_Scr_Item[]     = {"Độ lệch nhiệt độ.",
									"Giới hạn trên.   ",
									"Giới hạn dưới.   ",
									"Giới hạn ADC trên",
									"Giới hạn ADC dưới",
									"Thoát.           "};
const char *Param_Scr_Item[]     = {"Bước encoder.    ",
									"Lỗi tay hàn(ADC).",
									"Lỗi quá nhiệt.   ",
									"Ngưỡng nguồn cấp.",
									"Thoát.           "};
const char *PID_Scr_Item[]       = {"Độ lợi Kp.       ",
									"Độ lợi Ki.       ",
									"Độ lợi Kd.       ",
									"Đặt lại mặc định.",
									"Thoát.           "};
const char *RTC_Scr_Item[]       = {"Cài đặt ngày.    ",
									"Cài đặt giờ      ",
									"Thoát.           "};

const char *System_Information[]    = {" Designed by Mai", "Van Anh DHSPKT.", "Hardware Ver:2.1", "Firmware Ver:2.0", "Push to back."};

const char *Date_Str[]				= {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const char *Session_Str[]			= {"AM", "PM"};



uint8_t Pre_char_len = 1;
int8_t Pre_Select = 0;
int8_t HighLight  = 0;
int8_t First_Item = 0;

int8_t Pre_Select_Pre1 = 0;
int8_t HighLight_Pre1  = 0;
int8_t First_Item_Pre1 = 0;

int8_t Pre_Select_Pre2 = 0;
int8_t HighLight_Pre2  = 0;
int8_t First_Item_Pre2 = 0;

bool theme = false; //False=Light, True=Dark.

#define DARK_BG_COLOR        BLACK
#define DARK_TEXT_COLOR      WHITE
#define DARK_CONTAIN_COLOR   CYAN
#define DARK_TEMP_COLOR      GREEN //ORANGE
#define DARK_HEADER_COLOR    YELLOW
#define DARK_HEADER_BG_COLOR NAVY //BLUE

#define LIGHT_BG_COLOR        WHITE
#define LIGHT_TEXT_COLOR      0x101010 //BLACK
#define LIGHT_CONTAIN_COLOR   0x0085FF //BLUE
#define LIGHT_TEMP_COLOR      RED
#define LIGHT_HEADER_COLOR    WHITE
#define LIGHT_HEADER_BG_COLOR DARKGREEN

#define BG_COLOR        ((theme)? DARK_BG_COLOR:LIGHT_BG_COLOR)
#define TEXT_COLOR      ((theme)? DARK_TEXT_COLOR:LIGHT_TEXT_COLOR)
#define CONTAIN_COLOR   ((theme)? DARK_CONTAIN_COLOR:LIGHT_CONTAIN_COLOR)
#define TEMP_COLOR      ((theme)? DARK_TEMP_COLOR:LIGHT_TEMP_COLOR)
#define HEADER_COLOR    ((theme)? DARK_HEADER_COLOR :LIGHT_HEADER_COLOR )
#define HEADER_BG_COLOR ((theme)? DARK_HEADER_BG_COLOR:LIGHT_HEADER_BG_COLOR)


int8_t Option_Limit(int8_t Input, int8_t Bottom, int8_t Top);
int8_t Option_Limit_Invt(int8_t Input, int8_t Bottom, int8_t Top);
void write_num(uint16_t x, uint16_t y, uint16_t color, uint16_t b_color, uint16_t num);
void Print_Temperature(uint8_t x, uint8_t y, uint16_t Temperature, bool Unit);
void Draw_Menu(const char *Item_String[], uint8_t Num_Item, uint8_t Y_Start, int8_t Selecting);
void Show_Scr(parameter_t *param, const char *header);

int8_t Option_Limit(int8_t Input, int8_t Bottom, int8_t Top) {
	if(Input <= Bottom) return Bottom;
	if (Input >= Top) return Top;
	return Input;
}
int8_t Option_Limit_Invt(int8_t Input, int8_t Bottom, int8_t Top) {
	if(Input <= Bottom) return Top;
	if (Input >= Top) return Bottom;
	return Input;
}

void Menu_Restart(void){
	Encoder_Cnt = 0;
	Pre_Select = 0;
	HighLight  = 0;
	First_Item = 0;
}

void Menu_Save_State1(void){
	Pre_Select_Pre1 = Pre_Select;
	HighLight_Pre1  = HighLight;
	First_Item_Pre1 = First_Item;
}
void Menu_Return_State1(void){
	Pre_Select = Pre_Select_Pre1;
	HighLight  = HighLight_Pre1;
	First_Item = First_Item_Pre1;
}
void Menu_Save_State2(void){
	Pre_Select_Pre2 = Pre_Select;
	HighLight_Pre2  = HighLight;
	First_Item_Pre2 = First_Item;
}
void Menu_Return_State2(void){
	Pre_Select = Pre_Select_Pre2;
	HighLight  = HighLight_Pre2;
	First_Item = First_Item_Pre2;
}

void write_num(uint16_t x, uint16_t y, uint16_t color, uint16_t b_color, uint16_t num){
	uint8_t len;
	uint16_t x_start = x;
	uint16_t tmp = num;

	if(num<10) {
		len = 1;
		x_start = x;
	}
	else if(num >= 10 && num < 100) {
		len = 2;
		x_start = x+36;
	}
	else {
		len = 3;
		x_start = x+(36*2);
	}
	uint16_t draw_len = len*36+16+36;
	uint16_t clear_len = (LCD_WIDTH - draw_len)/2;

	if(len != Pre_char_len){
		tft.fill_rectangle(0, y, clear_len, 50, BG_COLOR);
		tft.fill_rectangle(LCD_WIDTH-clear_len-2, y, clear_len+2, 50, BG_COLOR);
		Pre_char_len = len;
	}

	for(uint8_t i=0; i<len; i++){
		tft.draw_MkE_bitmap(x_start-(i*36), y, 36, 50, color, b_color, Ubuntu36x50[tmp%10]);
		tmp = tmp/10;
	}

}

void Print_Temperature(uint8_t x, uint8_t y, uint16_t Temperature, bool Unit){
	uint8_t X = 0;
	uint8_t D_X = 0;
	if(Temperature<10){
		X = (LCD_WIDTH - (36*2+16)) / 2;
		D_X = X + 36;
	}
	else if(Temperature >= 10 && Temperature < 100){
		X = (LCD_WIDTH - (36*3+16)) / 2;
		D_X = X + 2*36;
	}
	else{
		X = (LCD_WIDTH - (36*4+16)) / 2;
		D_X = X + 3*36;
	}

//	tft.fill_rectangle(x, y, LCD_WIDTH, 50, BG_COLOR);

	write_num(X, y, TEMP_COLOR, BG_COLOR, Temperature);
	tft.draw_MkE_bitmap(D_X, y, 16, 50, TEMP_COLOR, BG_COLOR, Degree_Ubuntu16x50);
	(Unit == false)?
		tft.draw_MkE_bitmap(D_X+16, y, 36, 50, TEMP_COLOR, BG_COLOR, Ubuntu36x50[10]):
		tft.draw_MkE_bitmap(D_X+16, y, 36, 50, TEMP_COLOR, BG_COLOR, Ubuntu36x50[11]);
}

void Draw_Menu(const char *Item_String[], uint8_t Num_Item, uint8_t Y_Start, int8_t Selecting) {
	uint8_t Num_Print = ((128 - Y_Start) / 20) - 1; // 3
	int8_t brand;
	int8_t Select = Option_Limit(Selecting, 0, Num_Item);

	if (Select > Pre_Select){
		brand = Select - Pre_Select;
		HighLight += brand;
		if (HighLight > Num_Print){
			First_Item += brand;
		}
		Pre_Select = Select;
	}
	else if (Select < Pre_Select){
		brand = Pre_Select - Select;
		HighLight -= brand;
		if (HighLight < 0){
			First_Item -= brand;
		}
		Pre_Select = Select;
	}
	HighLight = Option_Limit(HighLight, 0, Num_Print);
	First_Item = Option_Limit(First_Item, 0, Num_Item - Num_Print - 1);

	tft.fill_rectangle(155, Y_Start, 4, (128 - Y_Start), BG_COLOR);
	tft.fill_rectangle(156, Y_Start, 2, (128 - Y_Start), TEXT_COLOR);

	uint8_t Y = Y_Start + HighLight*20;
	float Scroll = ((128 - Y_Start)) / (float)Num_Item;
	tft.fill_rectangle(155, Y_Start + (uint8_t)(Select * Scroll), 4, (uint8_t)Scroll + 2, TEXT_COLOR);

	tft.fill_rectangle(0, Y_Start, 1, (128 - Y_Start), BG_COLOR);
	tft.fill_rectangle(0, Y, 1, 20, TEXT_COLOR);
	uint8_t Start = First_Item;
	for(uint8_t i=0; i<=Num_Print; i++){
		uint16_t Color = TEXT_COLOR;
		uint16_t B_Color = BG_COLOR;
		if(i == HighLight) {
			Color = BG_COLOR;
			B_Color = TEXT_COLOR;
		}
		char item_Buf[40];
		sprintf(item_Buf, "%d.%s", First_Item + i+1, Item_String[Start++]);
		tft.print(1, Y_Start + i*20, Viet_Terminal8x20, Color, B_Color, (char *)item_Buf);
	}
}


void Show_Scr(parameter_t *param, const char *header){
	char Main_Buf[10];

	Print_Temperature(0, 78, param->temp, param->unit);

///* Hàng 3 */
	(param->unit)? sprintf(Main_Buf, "Set:%d@F", param->tempset) : sprintf(Main_Buf, "Set:%d@C", param->tempset);
	tft.print(4, 58, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);
	(param->unit)? sprintf(Main_Buf, "ITem:%d@F", param->intemp) : sprintf(Main_Buf, "ITem:%d@C", param->intemp);
	tft.print(84, 58, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);

///* Hàng 2 */
	sprintf(Main_Buf, "Vin:%.01fV", param->voltage);
	tft.print(4, 40, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);
	sprintf(Main_Buf, "Pow:%.01fW", param->power);
	tft.print(84, 40, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);

///* Hàng 1 */
	sprintf(Main_Buf, "%02d:%02d:%02d", param->time.hour, param->time.minutes, param->time.seconds);
	tft.print(4, 22, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);
	sprintf(Main_Buf, "%02d/%02d/%02d", param->time.dayofmonth, param->time.month, param->time.year);
	tft.print(84, 22, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);

///* Header */
	tft.print(0, 0, Viet_Terminal8x20, HEADER_COLOR, HEADER_BG_COLOR, (char *)header);
	tft.fill_rectangle(0, 20, LCD_WIDTH, 2, HEADER_BG_COLOR);

///* Ô 3 */
//	tft.draw_rect(0, 58, LCD_WIDTH, 18, CONTAIN_COLOR);
//	tft.fill_rectangle(80, 59, 1, 16, CONTAIN_COLOR);
///* Ô 2 */
//	tft.draw_rect(0, 40, LCD_WIDTH, 18, CONTAIN_COLOR);
//	tft.fill_rectangle(80, 41, 1, 16, CONTAIN_COLOR);
///* Ô 1 */
//	tft.draw_rect(0, 22, LCD_WIDTH, 18, CONTAIN_COLOR);
//	tft.fill_rectangle(80, 23, 1, 16, CONTAIN_COLOR);
}

void Heating_Scr(parameter_t *param){
	Show_Scr(param, Header_Screen_Item[1]);
}

void Sleeping_Scr(parameter_t *param){
	Show_Scr(param, Header_Screen_Item[2]);
}

void Temp_Set_Scr(parameter_t *param){
	char Main_Buf[40];

///* Header */
	tft.print(0, 0, Viet_Terminal8x20, HEADER_COLOR, HEADER_BG_COLOR, (char *)Header_Screen_Item[3]);
	tft.fill_rectangle(0, 20, LCD_WIDTH, 2, HEADER_BG_COLOR);

	Print_Temperature(0, 30, param->temp, param->unit);

	(param->unit)? sprintf(Main_Buf, "Hiện tại: %3d@F     ", param->temp): sprintf(Main_Buf, "Hiện tại: %3d@C     ", param->temp);
	tft.print(0, 80, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)Main_Buf);

	tft.print(0, 108, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)"Nhấn để lưu và thoát");
}

void Menu_Setting_Scr(int8_t Selecting){
///* Header */
	tft.print(0, 0, Viet_Terminal8x20, HEADER_COLOR, HEADER_BG_COLOR, (char *)Header_Screen_Item[0]);
	tft.fill_rectangle(0, 20, LCD_WIDTH, 2, HEADER_BG_COLOR);

	Draw_Menu(Menu_Scr_Item, 9, 22, Selecting);
}

void Menu_System_Scr(int8_t Selecting){
	///* Header */
	tft.print(0, 0, Viet_Terminal8x20, HEADER_COLOR, HEADER_BG_COLOR, (char *)Header_Screen_Item[0]);
	tft.fill_rectangle(0, 20, LCD_WIDTH, 2, HEADER_BG_COLOR);

	Draw_Menu(Menu_Scr_Item, 9, 22, Selecting);
}

//void Menu_Sleep_Scr(int8_t Selecting){
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(28, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"SLEEP");
//	lcd.drawRect(10, 1, 76, 13, Disp_Color);
//
//	Draw_Menu(Sleep_Scr_Item, 4, 16, Selecting);
//}
//
//void Menu_Calib_Scr(int8_t Selecting){
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(4, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"CALIBRATION");
//	lcd.drawRect(2, 1, 92, 13, Disp_Color);
//
//	Draw_Menu(Calib_Scr_Item, 6, 16, Selecting);
//}
//
//void Menu_Parameter_Scr(int8_t Selecting){
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(12, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"PARAMETER");
//	lcd.drawRect(10, 1, 76, 13, Disp_Color);
//
//	Draw_Menu(Parameter_Scr_Item, 5, 16, Selecting);
//}
//
//void Menu_PID_Scr(int8_t Selecting){
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(36, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"PID");
//	lcd.drawRect(10, 1, 76, 13, Disp_Color);
//
//	Draw_Menu(PID_Scr_Item, 5, 16, Selecting);
//}
//
//void Menu_RTC_Scr(int8_t Selecting){
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(12, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"REAL TIME");
//	lcd.print(28, 14, 8, 12, Terminal8x12, Disp_Color, (char *)"CLOCK");
//	lcd.drawRect(10, 1, 76, 25, Disp_Color);
//
//	Draw_Menu(RTC_Scr_Item, 3, 29, Selecting);
//}
//
//void Menu_Handle_Scr(uint16_t Temp_ADC, uint16_t Handle_Temp_ADC, bool Test, bool Rung){
//	char Label_Buf[12];
//
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(24, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"HANDLE");
//	lcd.drawRect(10, 1, 76, 13, Disp_Color);
//
//	if(Test == true) {
//		sprintf(Label_Buf, "State:%s", (char *)"Run");
//		lcd.print(12, 16, 8, 12, Terminal8x12, Disp_Color, (char *)Label_Buf);
//	}
//	else {
//		sprintf(Label_Buf, "State:%s", (char *)"Idle");
//		lcd.print(8, 16, 8, 12, Terminal8x12, Disp_Color, (char *)Label_Buf);
//	}
//
//	sprintf(Label_Buf, "1.%s:%d", (char *)"TipADC", Temp_ADC);
//	lcd.print(1, 29, 8, 12, Terminal8x12, Disp_Color, (char *)Label_Buf);
//
//	sprintf(Label_Buf, "2.%s:%d", (char *)"H_TADC", Handle_Temp_ADC);
//	lcd.print(1, 42, 8, 12, Terminal8x12, Disp_Color, (char *)Label_Buf);
//
//	sprintf(Label_Buf, "3.%s:", (char *)"Vibrate");
//	lcd.print(1, 55, 8, 12, Terminal8x12, Disp_Color, (char *)Label_Buf);
//
//	if(Rung) lcd.drawBitmap(83, 55, Up_Bitmap, Up_W, Up_H, Disp_Color);
//	else	 lcd.drawBitmap(83, 55, Down_Bitmap, Down_W, Down_H, Disp_Color);
//
//	lcd.Flushs();
//	lcd.Clear();
//}
//
//void Menu_Speaker_Scr(bool State){
//	Draw_State_Item_2(State, (char *)"BLUETOOTH", (char *)"SPEAKER", (char *)"ON", (char *)"OFF");
//}
//
//void Menu_SYSInfor_Scr(void){
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	lcd.print(24, 2, 8, 12, Terminal8x12, Disp_Color, (char *)"SYSTEM");
//	lcd.print(4, 15, 8, 12, Terminal8x12, Disp_Color, (char *)"INFORMATION");
//	lcd.drawRect(2, 1, 92, 26, Disp_Color);
//	uint8_t Y_Start = 28;
//	for(uint8_t i=0; i<5; i++){
//		lcd.print(1, Y_Start + i*8, Disp_Color, (char *)System_Information[i]);
//	}
//
//	lcd.Flushs();
//}
//
//
//void Draw_State_Item_1(bool State, char *Item, char *Option1, char *Option2){
//	char Label_Buf[10];
//
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	uint8_t len = strlen(Item);
//	uint8_t x = (96 - len * 8) / 2;
//	lcd.print(x, 2, 8, 12, Terminal8x12, Disp_Color, Item);
//	lcd.drawRect(x-2, 1, len*8 + 3, 13, Disp_Color);
//
//
//	if(State) {
//		len = strlen(Option1);
//		x = (96 - (len+2) * 8) / 2;
//		sprintf(Label_Buf, "[%s]", Option1);
//		lcd.print(x, 28, 8, 12, Terminal8x12, Disp_Color, Label_Buf);
//	}
//	else {
//		len = strlen(Option2);
//		x = (96 - (len+2) * 8) / 2;
//		sprintf(Label_Buf, "[%s]", Option2);
//		lcd.print(x, 28, 8, 12, Terminal8x12, Disp_Color, Label_Buf);
//	}
//	lcd.print(1, 55, 8, 12, Terminal8x12, Disp_Color, (char *)"Push to back");
//
//	lcd.Flushs();
//	lcd.Clear();
//}
//
//void Draw_State_Item_2(bool State, char *Item1, char *Item2, char *Option1, char *Option2){
//	char Label_Buf[10];
//
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	uint8_t len1 = strlen(Item1);
//	uint8_t x = (96 - len1 * 8) / 2;
//	lcd.print(x, 2, 8, 12, Terminal8x12, Disp_Color, Item1);
//	uint8_t len2 = strlen(Item2);
//	x = (96 - len2 * 8) / 2;
//	lcd.print(x, 15, 8, 12, Terminal8x12, Disp_Color, Item2);
//
//	if(len2 > len1) len1 = len2;
//	x = (96 - len1 * 8) / 2;
//	lcd.drawRect(x-2, 1, len1*8 + 3, 26, Disp_Color);
//
//	uint8_t len;
//	if(State) {
//		len = strlen(Option1);
//		x = (96 - (len+2) * 8) / 2;
//		sprintf(Label_Buf, "[%s]", Option1);
//		lcd.print(x, 35, 8, 12, Terminal8x12, Disp_Color, Label_Buf);
//	}
//	else {
//		len = strlen(Option2);
//		x = (96 - (len+2) * 8) / 2;
//		sprintf(Label_Buf, "[%s]", Option2);
//		lcd.print(x, 35, 8, 12, Terminal8x12, Disp_Color, Label_Buf);
//	}
//	lcd.print(1, 55, 8, 12, Terminal8x12, Disp_Color, (char *)"Push to back");
//
//	lcd.Flushs();
//	lcd.Clear();
//}
//
//void Draw_Number_Item_1(int16_t Number, char *Item, char * Unit){
//	char Label_Buf[10];
//
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	uint8_t len = strlen(Item);
//	uint8_t x = (96 - len * 8) / 2;
//	lcd.print(x, 2, 8, 12, Terminal8x12, Disp_Color, Item);
//	lcd.drawRect(x-2, 1, len*8 + 3, 13, Disp_Color);
//
//	sprintf(Label_Buf, "[%d%s]", Number, Unit);
//	len = strlen(Label_Buf);
//	x = (96 - len * 8) / 2;
//	lcd.print(x, 28, 8, 12, Terminal8x12, Disp_Color, Label_Buf);
//
//	lcd.print(1, 55, 8, 12, Terminal8x12, Disp_Color, (char *)"Push to back");
//
//	lcd.Flushs();
//	lcd.Clear();
//}
//
//void Draw_Number_Item_2(int16_t Number,  char *Item1, char *Item2, char * Unit){
//	char Label_Buf[10];
//
//	if(Disp_Color == false) lcd.fillScreen(true);
//
//	uint8_t len1 = strlen(Item1);
//	uint8_t x = (96 - len1 * 8) / 2;
//	lcd.print(x, 2, 8, 12, Terminal8x12, Disp_Color, Item1);
//	uint8_t len2 = strlen(Item2);
//	x = (96 - len2 * 8) / 2;
//	lcd.print(x, 15, 8, 12, Terminal8x12, Disp_Color, Item2);
//
//	if(len2 > len1) len1 = len2;
//	x = (96 - len1 * 8) / 2;
//	lcd.drawRect(x-2, 1, len1*8 + 3, 26, Disp_Color);
//
//	sprintf(Label_Buf, "[%d%s]", Number, Unit);
//	len1 = strlen(Label_Buf);
//	x = (96 - len1 * 8) / 2;
//	lcd.print(x, 35, 8, 12, Terminal8x12, Disp_Color, Label_Buf);
//
//	lcd.print(1, 55, 8, 12, Terminal8x12, Disp_Color, (char *)"Push to back");
//
//	lcd.Flushs();
//	lcd.Clear();
//}


