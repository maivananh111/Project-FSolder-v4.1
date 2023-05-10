/**
 * app_main.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"

#include "system/log.h"
#include "system/ret_err.h"
#include "system/system.h"

#include "periph/systick.h"
#include "periph/gpio.h"
#include "periph/dma.h"
#include "periph/spi.h"
#include "periph/tim.h"

#include "tftspi_st7735/tftspi_st7735.h"
#include "valconfig.h"


static const char *TAG = "Main";


TFTSPI_ST7735 tft(GPIOB, 14, GPIOB, 12);

volatile int16_t enc_val;

void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param);
void task_tim(void *);

void app_main(void){
	gpio_port_clock_enable(GPIOA);
	gpio_port_clock_enable(GPIOB);
	gpio_port_clock_enable(GPIOC);
	gpio_set_mode(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);
	gpio_set_mode(GPIOA, 11, GPIO_OUTPUT_PUSHPULL);

	gpio_set(GPIOA, 11);
	TFT_DMA->init(&dma1_channel5_conf);
	TFT_SPI->init(&spi2_conf);
	tft.init(TFT_SPI, TFT_COLOR_RGB);
	tft.set_rotation(1);
	tft.fill_screen(BLACK);
	gpio_reset(GPIOA, 11);
	tft.print(0, 0, Viet_Terminal8x20, YELLOW, BLACK, (char *)"Đmm Hoàng Anh");

	xTaskCreate(task_tim, "task_tim", byte_to_word(2048), NULL, 5, NULL);

	while(1){
		gpio_toggle(GPIOC, 13);
		LOG_INFO(TAG, "app_main task running...");
		vTaskDelay(5000);
	}
}

void task_tim(void *){
	ENC_TIM->init(&tim_enc_base_conf);
	ENC_TIM->register_event_handler(tim1_event_handler, NULL);
	ENC_TIM->set_mode_encoder(&tim_enc_conf);
	gpio_set_pullup(GPIOA, 8);
	gpio_set_pullup(GPIOA, 9);
	ENC_TIM->encoder_start_it();

	HA_TIM->init(&tim_ha_base_conf);
	HA_TIM->set_mode_pwm_output(TIM_CHANNEL2, &tim_ha_pwm_conf);
	gpio_remap(TIM2_Full_Remap);
	HA_TIM->pwm_output_start(TIM_CHANNEL2, 500);

	SD_TIM->init(&tim_sd_base_conf);
	SD_TIM->set_mode_pwm_output(TIM_CHANNEL1, &tim_sd_pwm_conf);
	gpio_remap(TIM3_Partial_Remap);
	SD_TIM->pwm_output_start(TIM_CHANNEL1, 500);

	while(1){
		char buf[10];
		sprintf(buf, "enc:%05d", enc_val);
		tft.print(0, 30, Viet_Terminal8x20, YELLOW, BLACK, (char *)buf);
		vTaskDelay(1);
	}
}

void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param){
	if(channel == TIM_CHANNEL1 || channel == TIM_CHANNEL2){
		if(event == TIM_EVENT_CAPTURECOMPARE1 || event == TIM_EVENT_CAPTURECOMPARE2)
			enc_val = ENC_TIM->encoder_get_counter();
	}
}
