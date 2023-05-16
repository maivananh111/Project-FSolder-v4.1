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
#include "spiflash/spiflash.h"
#include "kalman_filter/kalman_filter.h"
#include "pid/pid.h"
#include "button/button.h"
#include "valconfig.h"
#include "gui/gui.h"

static const char *TAG = "Main";

TFTSPI_ST7735 tft(GPIOB, 14, GPIOB, 12);
SPIFLASH flash(GPIOB, 2);
#define FLASH_PARAM_STORAGE_ADDRESS 0x1000U

/* System variable */
volatile int16_t Encoder_Cnt;
volatile uint32_t Encoder_Val;
uint16_t adc_val[6];

/* Heat control variable */
pid sd_pid;
kalmanfilter sd_temp_filter(5, 5, 50);

/* GUI variable */
EventGroupHandle_t enc_button_event = NULL;
#define SINGLE_CLICK_BIT (1<<10)
#define DOUBLE_CLICK_BIT (1<<11)
#define LONG_PRESS_BIT   (1<<12)




void task_sys_process(void *);
void task_display(void *);

void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param);
void exti_line10_handler(void *);
void enc_button_event_handler(button_event_t);

void read_storage_param(system_param_t *parameter);
void write_storage_param(system_param_t *parameter);

void app_main(void){
	gpio_port_clock_enable(GPIOA);
	gpio_port_clock_enable(GPIOC);
	gpio_set_mode(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);

	xTaskCreate(task_sys_process, "task_sys_process", byte_to_word(1024), NULL, 10, NULL);
	xTaskCreate(task_display, "task_display", byte_to_word(2048), NULL, 5, NULL);

	while(1){
		gpio_toggle(GPIOC, 13);
		DS3231_GetTime(&(show_data.time));
		vTaskDelay(500);
	}
}

void task_sys_process(void *){
	/**
	 * Peripheral initialize.
	 */
	gpio_set_mode(GPIOA, 11, GPIO_OUTPUT_PUSHPULL);
	gpio_set(GPIOA, 11);

	ENC_TIM->init(&tim_enc_base_conf);
	ENC_TIM->register_event_handler(tim1_event_handler, NULL);
	ENC_TIM->set_mode_encoder(&tim_enc_conf);
	gpio_set_pullup(GPIOA, 8);
	gpio_set_pullup(GPIOA, 9);
	ENC_TIM->encoder_start_it();

	HA_TIM->init(&tim_ha_base_conf);
	HA_TIM->set_mode_pwm_output(TIM_CHANNEL1, &tim_ha_pwm_conf);
	gpio_remap(TIM2_Full_Remap);
	HA_TIM->pwm_output_start(TIM_CHANNEL1, 500);

	SD_TIM->init(&tim_sd_base_conf);
	SD_TIM->set_mode_pwm_output(TIM_CHANNEL2, &tim_sd_pwm_conf);
	gpio_remap(TIM3_Partial_Remap);
	SD_TIM->pwm_output_start(TIM_CHANNEL2, 999);

	ADC_DMA->init(&dma1_channel1_conf);
	ADC_ADC->init(&adc1_conf);
	ADC_ADC->start_dma(adc_val, 6);

	dma1_channel2->init(&dma1_channel2_conf);
	dma1_channel3->init(&dma1_channel3_conf);
	spi1->init(&spi1_conf);
	uint32_t ID = flash.Init(spi1);
	LOG_WARN(TAG, "Flash Initialize finish, ID: 0x%08x.", ID);

	RTC_I2C->init(&i2c1_conf);
	gpio_remap(I2C1_Remap);
	DS3231_Init(i2c1);
	rtc_time_t stime = {
		.seconds = 0,
		.minutes = 14,
		.hour = 17,
		.dayofweek = 3,
		.dayofmonth = 16,
		.month = 5,
		.year = 23,
	};
	DS3231_SetTime(stime);

	gpio_reset(GPIOA, 11);

	/**
	 * System features initialize.
	 */
	sd_pid.init(&param.sd_pid_param);
	sd_pid.set_point(show_data.tempset);

	while(1){
		show_data.temp = sd_temp_filter.update_estimate(adc_val[0], 30);
//		show_data.power = sd_temp_filter.update_estimate(adc_val[1], 30);
//		show_data.voltage = sd_temp_filter.update_estimate(adc_val[4], 30);
//		show_data.intemp = sd_temp_filter.update_estimate(adc_val[5], 30);
		vTaskDelay(100);
	}
}

void task_display(void *){
	TFT_DMA->init(&dma1_channel5_conf);
	TFT_SPI->init(&spi2_conf);
	tft.init(TFT_SPI, TFT_COLOR_RGB);
	tft.set_rotation(1);
	gui_clear();

	button_init(&enc_button);
	exti_register_event_handler(10, exti_line10_handler, NULL);
	enc_button_event = xEventGroupCreate();

	while(1){
		button_task_handler(&enc_button);

		EventBits_t bits = xEventGroupWaitBits(enc_button_event,
				(SINGLE_CLICK_BIT | DOUBLE_CLICK_BIT | LONG_PRESS_BIT), pdTRUE, pdFALSE, 5);

		switch(gui_main_activite){
			case HEATING:
				Heating_Scr(&show_data);
				/**
				 * Heating handle.
				 */
				if(bits != 0){
					if     (bits == SINGLE_CLICK_BIT) gui_main_activite = SETTING; // Đến giao diện đặt nhiệt độ.
					else if(bits == DOUBLE_CLICK_BIT) gui_main_activite = SLEEPING;// Đến giao diện ngủ.
					else if(bits == LONG_PRESS_BIT) {
						gui_main_activite = MENU;      // Đến giao diện menu.
						Encoder_Cnt = 0;
						tim1->encoder_set_counter(0);
					}
					gui_clear();
				}
			break;
			case SLEEPING:
				Sleeping_Scr(&show_data);
				/**
				 * Sleep handle.
				 */
				if(bits && (SINGLE_CLICK_BIT | DOUBLE_CLICK_BIT | LONG_PRESS_BIT)){
					gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
					gui_clear();
				}
			break;
			case SETTING:
				Temp_Set_Scr(&show_data);
				if(bits == SINGLE_CLICK_BIT){
					gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
					/**
					 * Save new temperature set value.
					 */
					gui_clear();
				}
			break;
			case MENU:
				Menu_Setting_Scr((int8_t *)&Encoder_Cnt);
				if((bits == LONG_PRESS_BIT)){
					gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
					gui_clear();
				}
				if(bits == SINGLE_CLICK_BIT){
					switch(Encoder_Cnt){
						case 0:

						break;
						case 1:

						break;
						case 2:

						break;
						case 3:

						break;
						case 4:

						break;
						case 5:

						break;
						case 6:

						break;
						case 7:

						break;
						case 8:
							gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
							gui_clear();
						break;
					}
				}

			break;
		}




//		Menu_System_Scr((int8_t *)&Encoder_Cnt);
//		Menu_Sleep_Scr((int8_t *)&Encoder_Cnt);
//		Menu_Calib_Scr((int8_t *)&Encoder_Cnt);
//		Menu_PID_Scr((int8_t *)&Encoder_Cnt);
//		Menu_RTC_Scr((int8_t *)&Encoder_Cnt);
//		Menu_SYSInfor_Scr();
		vTaskDelay(5);
	}
}


void exti_line10_handler(void *){
	button_interrupt_trigger(&enc_button);
}
void enc_button_event_handler(button_event_t event){
	gpio_set(GPIOA, 11);
	switch(event){
		case BUTTON_SINGLECLICK:
			xEventGroupSetBits(enc_button_event, SINGLE_CLICK_BIT);
		break;
		case BUTTON_DOUBLECLICK:
			xEventGroupSetBits(enc_button_event, DOUBLE_CLICK_BIT);
		break;
		case BUTTON_LONGPRESS:
			xEventGroupSetBits(enc_button_event, LONG_PRESS_BIT);
		break;
		default:
		break;
	}
	LOG_ERROR(TAG, "%d", Encoder_Cnt);
	vTaskDelay(10);
	gpio_reset(GPIOA, 11);
}


void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param){
	if(channel == TIM_CHANNEL1 || channel == TIM_CHANNEL2){
		if(event == TIM_EVENT_CAPTURECOMPARE1 || event == TIM_EVENT_CAPTURECOMPARE2){
			Encoder_Cnt = tim1->encoder_get_counter();
			if(Encoder_Cnt > gui_get_top()){
				Encoder_Cnt = gui_get_top();
				tim1->encoder_set_counter(gui_get_top());
			}

			if(Encoder_Cnt < gui_get_bottom()){
				Encoder_Cnt = gui_get_bottom();
				tim1->encoder_set_counter(gui_get_bottom());
			}
		}
	}
}


void read_storage_param(system_param_t *parameter){
	char *Rxbuf = (char *)malloc(37*sizeof(uint8_t));
	flash.ReadBytes(FLASH_PARAM_STORAGE_ADDRESS, (uint8_t *)Rxbuf, 37);

	/** System */
	parameter->control_type  = Rxbuf[0];
	parameter->enable_buzzer = Rxbuf[1];
	parameter->gui_theme     = Rxbuf[2];
	/** Sleep */
	parameter->sleep_temp    = (Rxbuf[3]<<8) | Rxbuf[4];
	parameter->sleep_wait_time = (Rxbuf[5]<<8) | Rxbuf[6];
	parameter->enable_weakup = Rxbuf[7];
	/** Calibration */
	parameter->temp_offset   = (Rxbuf[8]<<8) | Rxbuf[9];
	parameter->temp_max      = (Rxbuf[10]<<8) | Rxbuf[11];
	parameter->temp_min      = (Rxbuf[12]<<8) | Rxbuf[13];
	parameter->adc_max       = (Rxbuf[14]<<8) | Rxbuf[15];
	parameter->adc_min       = (Rxbuf[16]<<8) | Rxbuf[17];
	/** Parameter */
	parameter->encoder_step  = Rxbuf[18];
	parameter->adc_error     = (Rxbuf[19]<<8) | Rxbuf[20];
	parameter->over_temp     = (Rxbuf[21]<<8) | Rxbuf[22];
	parameter->voltage_threshold = (Rxbuf[23]<<8) | Rxbuf[24];
	/** PID */
	parameter->sd_pid_param.direction = (pid_direction_t)Rxbuf[25];
	parameter->sd_pid_param.kp = (Rxbuf[26]<<8) | Rxbuf[27];
	parameter->sd_pid_param.ki = (Rxbuf[28]<<8) | Rxbuf[29];
	parameter->sd_pid_param.kd = (Rxbuf[30]<<8) | Rxbuf[31];
	parameter->sd_pid_param.max_output = (Rxbuf[32]<<8) | Rxbuf[33];
	parameter->sd_pid_param.min_output = (Rxbuf[34]<<8) | Rxbuf[35];
	parameter->sd_pid_param.sample_time = (Rxbuf[36]<<8) | Rxbuf[37];

	free(Rxbuf);
}

void write_storage_param(system_param_t *parameter){
	char *Data = (char *)malloc(37*sizeof(uint8_t));

	/** System */
	Data[0] = parameter->control_type;
	Data[1] = parameter->enable_buzzer;
	Data[2] = parameter->gui_theme;
	/** Sleep */
	Data[3] = (parameter->sleep_temp >> 8);
	Data[4] = (parameter->sleep_temp & 0xFF);
	Data[5] = (parameter->sleep_wait_time >> 8);
	Data[6] = (parameter->sleep_wait_time & 0xFF);
	Data[7] = parameter->enable_weakup;
	/** Calibration */
	Data[8] = (parameter->temp_offset >> 8);
	Data[9] = (parameter->temp_offset & 0xFF);
	Data[10] = (parameter->temp_max >> 8);
	Data[11] = (parameter->temp_max & 0xFF);
	Data[12] = (parameter->temp_min >> 8);
	Data[13] = (parameter->temp_min & 0xFF);
	Data[14] = (parameter->adc_max >> 8);
	Data[15] = (parameter->adc_max & 0xFF);
	Data[16] = (parameter->adc_min >> 8);
	Data[17] = (parameter->adc_min & 0xFF);
	/** Parameter */
	Data[18] = parameter->encoder_step;
	Data[19] = (parameter->adc_error >> 8);
	Data[20] = (parameter->adc_error & 0xFF);
	Data[21] = (parameter->over_temp >> 8);
	Data[22] = (parameter->over_temp & 0xFF);
	Data[23] = (uint8_t)(parameter->voltage_threshold);
	Data[24] = (int)(parameter->voltage_threshold*100)%100;
	/** PID */
	Data[25] = parameter->sd_pid_param.direction;
	Data[26] = (uint8_t)(parameter->sd_pid_param.kp);
	Data[27] = (int)(parameter->sd_pid_param.kp*100)%100;
	Data[28] = (uint8_t)(parameter->sd_pid_param.ki);
	Data[29] = (int)(parameter->sd_pid_param.ki*100)%100;
	Data[30] = (uint8_t)(parameter->sd_pid_param.kd);
	Data[31] = (int)(parameter->sd_pid_param.kd*100)%100;
	Data[32] = (uint8_t)(parameter->sd_pid_param.max_output);
	Data[33] = (int)(parameter->sd_pid_param.max_output*100)%100;
	Data[34] = ((uint8_t)parameter->sd_pid_param.min_output);
	Data[35] = (int)(parameter->sd_pid_param.min_output*100)%100;
	Data[36] = (uint8_t)(parameter->sd_pid_param.sample_time);
	Data[37] = (int)(parameter->sd_pid_param.sample_time*100)%100;

	flash.WriteBytes(FLASH_PARAM_STORAGE_ADDRESS, (uint8_t *)Data, 37);

	free(Data);
}



