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
#include "event_groups.h"

#include "system/log.h"
#include "system/ret_err.h"
#include "system/system.h"

#include "periph/systick.h"
#include "periph/gpio.h"
#include "periph/exti.h"
#include "periph/dma.h"
#include "periph/spi.h"
#include "periph/tim.h"

#include "tftspi_st7735/tftspi_st7735.h"
#include "pid/pid.h"
#include "button/button.h"
#include "valconfig.h"
#include "gui/gui.h"

static const char *TAG = "Main";

TFTSPI_ST7735 tft(GPIOB, 14, GPIOB, 12);

volatile int16_t Encoder_Cnt;
uint16_t adc_val[6];

void task_sys_process(void *);
void task_display(void *);
void task_gui_handler(void *);

void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param);
void exti_line10_handler(void *);
void enc_button_event_handler(button_event_t);

void app_main(void){
	gpio_port_clock_enable(GPIOA);
	gpio_port_clock_enable(GPIOC);
	gpio_set_mode(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);

	xTaskCreate(task_sys_process, "task_sys_process", byte_to_word(1024), NULL, 10, NULL);
	xTaskCreate(task_display, "task_display", byte_to_word(1024), NULL, 5, NULL);
	xTaskCreate(task_gui_handler, "task_gui_handler", byte_to_word(1024), NULL, 8, NULL);

	while(1){
		gpio_toggle(GPIOC, 13);
		vTaskDelay(1000);
	}
}

void task_sys_process(void *){
	/**
	 * Peripheral initialize.
	 */
	gpio_set_mode(GPIOA, 11, GPIO_OUTPUT_PUSHPULL);

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

	ADC_DMA->init(&dma1_channel1_conf);
	ADC_ADC->init(&adc1_conf);
	ADC_ADC->start_dma(adc_val, 6);

	/**
	 * System features initialize.
	 */
	sd_pid.init(&sd_pid_param);
	sd_pid.set_point(sd_temp_set);

	while(1){
		show_data.temp = Encoder_Cnt;

		vTaskDelay(100);
	}
}

void task_display(void *){
	TFT_DMA->init(&dma1_channel5_conf);
	TFT_SPI->init(&spi2_conf);
	tft.init(TFT_SPI, TFT_COLOR_RGB);
	tft.set_rotation(1);
	tft.fill_screen(WHITE);

	while(1){
		Heating_Scr(&show_data);
//		Temp_Set_Scr(&show_data);
//		Menu_Setting_Scr(Encoder_Cnt);


		vTaskDelay(5);
	}
}

void task_gui_handler(void *){
	button_init(&enc_button);
	exti_register_event_handler(10, exti_line10_handler, NULL);

	while(1){
		button_task_handler(&enc_button);
		vTaskDelay(50);
	}
}


void exti_line10_handler(void *){
	button_interrupt_trigger(&enc_button);
}
void enc_button_event_handler(button_event_t event){
	gpio_set(GPIOA, 11);
	switch(event){
		case BUTTON_SINGLECLICK:
			LOG_WARN(TAG, "Button single click.");
		break;
		case BUTTON_DOUBLECLICK:
			LOG_WARN(TAG, "Button double click.");
		break;
		case BUTTON_LONGPRESS:
			LOG_WARN(TAG, "Button long press.");
		break;
		default:
		break;
	}
	vTaskDelay(10);
	gpio_reset(GPIOA, 11);
}


void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param){
	if(channel == TIM_CHANNEL1 || channel == TIM_CHANNEL2){
		if(event == TIM_EVENT_CAPTURECOMPARE1 || event == TIM_EVENT_CAPTURECOMPARE2)
			Encoder_Cnt = ENC_TIM->encoder_get_counter();
	}
}
