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
#include "pid/pid.hpp"
#include "kalman_filter/kalman_filter.h"
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
uint8_t adc_cnt = 0;
uint16_t adc_buffer[6];
uint32_t adc_val[6];

float vin_a = 0.0117195122F;
float vin_b = -0.7486585366F;
float itemp_a = 0.08536585366F;
float itemp_b = -130.6097561F;
float temp_a = 0.3790322581F;
float temp_b = -601.0887097F;


/* Heat control variable */
pid<int16_t> sd_pid;
kalmanfilter sd_temp_filter(10, 10, 50);
#define KALMAN_LOOP 50

/* GUI variable */
EventGroupHandle_t enc_button_event = NULL;
#define SINGLE_CLICK_BIT (1<<10)
#define DOUBLE_CLICK_BIT (1<<11)
#define LONG_PRESS_BIT   (1<<12)
main_activite_t gui_lmain_activite;
uint16_t vibra_stt = 0, lvibra_stt = 1;

void task_sys_process(void *);
void task_display(void *);

void tim1_event_handler(tim_channel_t channel, tim_event_t event, void *param);
void sd_vibra_weakup_handler(void *);
void exti_line10_handler(void *);
void enc_button_event_handler(button_event_t);
void DMA1_Channel2_CallBack(dma_event_t event, void *Parameter);
void DMA1_Channel3_CallBack(dma_event_t event, void *Parameter);

void read_storage_param(system_param_t *parameter);
void write_storage_param(system_param_t *parameter);

void beep(uint8_t loop);

void app_main(void){
	gpio_port_clock_enable(GPIOA);
	gpio_port_clock_enable(GPIOC);
	gpio_set_mode(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);
	TFT_DMA->init(&dma1_channel5_conf);
	TFT_SPI->init(&spi2_conf);
	tft.init(TFT_SPI, TFT_COLOR_RGB);
	tft.set_rotation(1);
	gui_clear();
	tft.print(0, 50, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)"  Đang khởi động...");

	xTaskCreate(task_sys_process, "task_sys_process", byte_to_word(2048), NULL, 10, NULL);

	while(1){
		gpio_toggle(GPIOC, 13);
		DS3231_GetTime(&(show_data.time));

		if(show_data.time.seconds != sys_param.l_seconds && gui_main_activite == HEATING){ // Check sleep mode.
			sys_param.l_seconds = show_data.time.seconds;
			sys_param.sleep_tick_count++;
			LOG_DEBUG(TAG, "%d", sys_param.sleep_tick_count);
			if(sys_param.sleep_tick_count > sys_param.sleep_wait_time){
				gui_main_activite = SLEEPING;
				sys_param.sleep_tick_count = 0;
				sys_param.tempset_before_sleep = sys_param.temp_set;
				sys_param.temp_set = sys_param.sleep_temp_set;
				LOG_INFO(TAG, "%d", sys_param.temp_set);
				beep(5);
			}
		}

		vTaskDelay(500);
	}
}

void task_sys_process(void *){
	/**
	 * Peripheral initialize.
	 */
	RTC_I2C->init(&i2c1_conf);
	gpio_remap(I2C1_Remap);
	DS3231_Init(i2c1);

	gpio_set_mode(GPIOA, 11, GPIO_OUTPUT_PUSHPULL);
	beep(2);
	gpio_set_mode(GPIOA, 2, GPIO_INPUT_PULLUP);
//	exti_init(GPIOA, 2, EXTI_FALLING_EDGE, 8);
//	exti_register_event_handler(2, sd_vibra_weakup_handler, NULL);

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
	SD_TIM->pwm_output_start(TIM_CHANNEL2, 1);

	ADC_DMA->init(&dma1_channel1_conf);
	ADC_ADC->init(&adc1_conf);
	ADC_ADC->start_dma(adc_buffer, 6);

	dma1_channel2->init(&dma1_channel2_conf);
	dma1_channel2->register_event_handler(DMA1_Channel2_CallBack, NULL);
	dma1_channel3->init(&dma1_channel3_conf);
	dma1_channel3->register_event_handler(DMA1_Channel3_CallBack, NULL);
	spi1->init(&spi1_conf);
	uint32_t ID = flash.Init(spi1);
	LOG_WARN(TAG, "Flash Initialize finish, ID: 0x%08x.", ID);
	read_storage_param(&sys_param);


/*	rtc_time_t stime = {
		.seconds = 0,
		.minutes = 54,
		.hour = 23,
		.dayofweek = 4,
		.dayofmonth = 17,
		.month = 5,
		.year = 23,
	};
	DS3231_SetTime(stime);
*/

	gui_clear();

	xTaskCreate(task_display, "task_display", byte_to_word(4096), NULL, 5, NULL);
	beep(1);

	/**
	 * System features initialize.
	 */
	sys_param.sd_pid_param.direction = PID_NORMAL;
	sys_param.sd_pid_param.kp = 15.0;
	sys_param.sd_pid_param.ki = 1.8;
	sys_param.sd_pid_param.kd = 0.5,
	sys_param.sd_pid_param.max_output = 499;
	sys_param.sd_pid_param.min_output = 0;
	sys_param.sd_pid_param.sample_time = 100U;
	sd_pid.init(&sys_param.sd_pid_param);

	while(1){
//		show_data.tempset = sys_param.temp_set;
		sd_pid.set_point(sys_param.temp_set);
		if(adc_cnt < 20){
			for(uint8_t i=0; i<6; i++)
				adc_val[i] += adc_buffer[i];
			adc_cnt++;
		}
		else{
			for(uint8_t i=0; i<6; i++)
				adc_val[i] /= 20;
			adc_cnt = 0;
			show_data.temp = temp_a * adc_val[0] + temp_b;

			sys_param.sd_pwm = (uint16_t)sd_pid.calculate(show_data.temp);
			SD_TIM->pwm_output_set_duty(TIM_CHANNEL2, sys_param.sd_pwm);

			show_data.power = (float)adc_val[1]/5.0;

			show_data.voltage = vin_a * adc_val[4] + vin_b;
			show_data.intemp = itemp_a * adc_val[5] + itemp_b;
		}

		vTaskDelay(5);
	}
}

void task_display(void *){
	button_init(&enc_button);
	exti_register_event_handler(10, exti_line10_handler, NULL);
	enc_button_event = xEventGroupCreate();

	show_data.tempset = sys_param.temp_set;

	while(1){
		button_task_handler(&enc_button);

		EventBits_t bits = xEventGroupWaitBits(enc_button_event,
				(SINGLE_CLICK_BIT | DOUBLE_CLICK_BIT | LONG_PRESS_BIT), pdTRUE, pdFALSE, 5);

		switch(gui_main_activite){
			case HEATING:
				Heating_Scr(&show_data);

				show_data.tempset = sys_param.temp_set;
				if(show_data.temp > sys_param.temp_error || show_data.voltage < sys_param.voltage_threshold){
					beep(3);
					gui_main_activite = SD_ERROR;
					gui_lmain_activite = HEATING;
					gui_clear();
				}

				vibra_stt = gpio_get_level(GPIOA, 2);
				if(vibra_stt == 0 && lvibra_stt != 0){
					sys_param.sleep_tick_count = 0;
					lvibra_stt = vibra_stt;
					LOG_WARN(TAG, "ngắt");
				}
				else if(vibra_stt == 1 && lvibra_stt == 0){
					lvibra_stt = 1;
				}
				/**
				 * Heating handle.
				 */
				if(bits != 0){
					if     (bits == SINGLE_CLICK_BIT){
						gui_main_activite = SETTING; // Đến giao diện đặt nhiệt độ.
						gui_limit_counter(sys_param.temp_min, sys_param.temp_max);
						Encoder_Cnt = sys_param.temp_set;
						tim1->encoder_set_counter(sys_param.temp_set);
						LOG_INFO(TAG, "Switch to SETTING, line %d", __LINE__);
					}
					else if(bits == DOUBLE_CLICK_BIT){
						gui_main_activite = SLEEPING;// Đến giao diện ngủ.
						sys_param.temp_set = sys_param.sleep_temp_set;
						sys_param.force_sleep = true;
						LOG_INFO(TAG, "Switch to SLEEPING, line %d", __LINE__);
					}
					else if(bits == LONG_PRESS_BIT) {
						gui_main_activite = MENU;      // Đến giao diện menu.
						LOG_INFO(TAG, "Switch to MENU, line %d", __LINE__);
						Encoder_Cnt = 0;
						tim1->encoder_set_counter(0);
					}
					gui_clear();
				}
			break;
			case SLEEPING:
				Sleeping_Scr(&show_data);
				show_data.tempset = sys_param.sleep_temp_set;
				if(show_data.temp > sys_param.temp_error || show_data.voltage < sys_param.voltage_threshold){
					beep(3);
					gui_main_activite = SD_ERROR;
					gui_lmain_activite = HEATING;
					gui_clear();
				}
				vibra_stt = gpio_get_level(GPIOA, 2);
				if(vibra_stt == 0 && lvibra_stt != 0){
					sys_param.sleep_tick_count = 0;
					if(sys_param.force_sleep == false && sys_param.enable_weakup == true){
						gui_main_activite = HEATING;
						sys_param.temp_set = sys_param.tempset_before_sleep;
					}
					LOG_WARN(TAG, "ngắt");

				}
				else if(vibra_stt == 1 && lvibra_stt == 0){
					lvibra_stt = 1;
				}
				/**
				 * Sleep handle.
				 */
				if(bits && (SINGLE_CLICK_BIT | DOUBLE_CLICK_BIT | LONG_PRESS_BIT)){
					gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
					sys_param.force_sleep = false;
					sys_param.temp_set = sys_param.tempset_before_sleep;
					write_storage_param(&sys_param);
					LOG_INFO(TAG, "Switch to HEATING, line %d", __LINE__);
					gui_clear();
				}
			break;
			case SETTING:
				gui_limit_counter(sys_param.temp_min, sys_param.temp_max);
				show_data.tempset = Encoder_Cnt;
				Temp_Set_Scr(&show_data);
				if(bits == SINGLE_CLICK_BIT){
					gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
					LOG_INFO(TAG, "Switch to HEATING, line %d", __LINE__);
					/**
					 * Save new temperature set value.
					 */

					sys_param.temp_set = show_data.tempset;
					write_storage_param(&sys_param);

					gui_clear();
				}
			break;
			case MENU:{
				/**
				 * Show menu.
				 */
				if(gui_menu_layer == MENU_LAYER_1){
					Menu_Setting_Scr((int8_t *)&Encoder_Cnt);
				}
				else if(gui_menu_layer == MENU_LAYER_2){
					switch(gui_menu_layer2_num){
						case 0:
							gui_limit_counter(sys_param.temp_min, sys_param.temp_max);
							show_data.tempset = Encoder_Cnt;
							Temp_Set_Scr(&show_data);

						break;
						case 1:
							Menu_System_Scr((int8_t *)&Encoder_Cnt);
						break;
						case 2:
							Menu_Sleep_Scr((int8_t *)&Encoder_Cnt);
						break;
						case 3:
							Menu_Calib_Scr((int8_t *)&Encoder_Cnt);
						break;
						case 4:
							Menu_Param_Scr((int8_t *)&Encoder_Cnt);
						break;
						case 5:
							Menu_PID_Scr((int8_t *)&Encoder_Cnt);
						break;
						case 6:
							Menu_RTC_Scr((int8_t *)&Encoder_Cnt);
						break;
						case 7:
							Menu_SYSInfor_Scr();
						break;
						case 8:
							gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
							LOG_INFO(TAG, "Switch to HEATING, line %d", __LINE__);
							gui_menu_layer = MENU_LAYER_1;
							gui_menu_layer2_num = 0;
							Encoder_Cnt = gui_menu_layer2_num;
							tim1->encoder_set_counter(gui_menu_layer2_num);
							write_storage_param(&sys_param);
						break;
					}
				}
				else if(gui_menu_layer == MENU_LAYER_3){
					tft.print(0,  0, Viet_Terminal8x20, TEXT_COLOR, BG_COLOR, (char *)"Layer 3");
				}

				/**
				 * Check encoder event.
				 */
				if((bits == LONG_PRESS_BIT)){
					if(gui_menu_layer == MENU_LAYER_1){ // Giữ lúc ở menu cấp 1(chính) thì quay về trạng thái đang chạy.
						gui_main_activite = HEATING; // Đến giao diện Đang hoạt động.
						LOG_INFO(TAG, "Switch to HEATING, line %d", __LINE__);
					}
					else if(gui_menu_layer == MENU_LAYER_2){ // Giữ lúc ở menu cấp 2 thì quay về menu cấp 1(chính).
						gui_menu_layer = MENU_LAYER_1;
						LOG_INFO(TAG, "Switch to MENU_LAYER_1, line %d", __LINE__);
						Encoder_Cnt = gui_menu_layer2_num;
						tim1->encoder_set_counter(gui_menu_layer2_num);
						if(gui_menu_layer2_num == 0){
							sys_param.temp_set = show_data.tempset;
						}
					}
					gui_clear();
				}
				if(bits == SINGLE_CLICK_BIT){
					if(gui_menu_layer == MENU_LAYER_1){ // Nhấn ở menu cấp 1(chính) để vào menu cấp 2.
						gui_menu_layer = MENU_LAYER_2;
						LOG_INFO(TAG, "Switch to MENU_LAYER_2 num %d, line %d", gui_menu_layer2_num, __LINE__);
						gui_menu_layer2_num = Encoder_Cnt;
						Encoder_Cnt = 0;
						tim1->encoder_set_counter(0);
						if(gui_menu_layer2_num == 0){
							Encoder_Cnt = sys_param.temp_set;
							tim1->encoder_set_counter(sys_param.temp_set);
						}
						gui_clear();
					}
					else if(gui_menu_layer == MENU_LAYER_2){ // Nhấn ở menu cấp 2 để vào nenu cấp 3.
						if(gui_menu_layer2_num == 0){ // Nếu là set nhiệt độ thì quay về menu chính chứ không vào menu cấp 3.
							gui_menu_layer = MENU_LAYER_1;
							LOG_INFO(TAG, "Switch to MENU_LAYER_1, line %d", __LINE__);
							Encoder_Cnt = gui_menu_layer2_num;
							tim1->encoder_set_counter(gui_menu_layer2_num);
							sys_param.temp_set = show_data.tempset;
							gui_clear();
						}
						else{
							gui_menu_layer = MENU_LAYER_3;
							LOG_INFO(TAG, "Switch to MENU_LAYER_3, line %d", __LINE__);
							gui_menu_layer3_num = Encoder_Cnt;
							Encoder_Cnt = 0;
							tim1->encoder_set_counter(0);
						}
					}
					else if(gui_menu_layer == MENU_LAYER_3){ // Nhấn ở menu cấp 1(chính) để vào menu cấp 2.
						gui_menu_layer = MENU_LAYER_2;
						gui_menu_layer2_num = Encoder_Cnt;
						LOG_INFO(TAG, "Switch to MENU_LAYER_2 num %d, line %d", gui_menu_layer2_num, __LINE__);
						Encoder_Cnt = 0;
						tim1->encoder_set_counter(0);
						if(gui_menu_layer2_num == 0){
							Encoder_Cnt = sys_param.temp_set;
							tim1->encoder_set_counter(sys_param.temp_set);
						}
						gui_clear();
					}
				}
			}
			break;
			case SD_ERROR:

				if((show_data.temp < sys_param.temp_error) && (show_data.voltage < sys_param.voltage_threshold)){
					Error_Scr((char *)"   Nguồn cấp yếu.   ");
				}
				if((show_data.temp > sys_param.temp_error) && (show_data.voltage > sys_param.voltage_threshold)){
					Error_Scr((char *)" Chưa gắn tay hàn.  ");
				}
				if((show_data.temp < sys_param.temp_error) && (show_data.voltage > sys_param.voltage_threshold)){
					gui_main_activite = gui_lmain_activite;
					gui_clear();
					beep(1);
				}
			break;
		}

		vTaskDelay(5);
	}
}


void sd_vibra_weakup_handler(void *){
	sys_param.sleep_tick_count = 0;
	if(gui_main_activite == SLEEPING && sys_param.force_sleep == false && sys_param.enable_weakup == true){
		gui_main_activite = HEATING;
		sys_param.temp_set = sys_param.tempset_before_sleep;
	}
	LOG_WARN(TAG, "ngắt");
}

void exti_line10_handler(void *){
	button_interrupt_trigger(&enc_button);
}
void enc_button_event_handler(button_event_t event){
	if(sys_param.enable_buzzer) gpio_set(GPIOA, 11);
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
	if(sys_param.enable_buzzer){
		vTaskDelay(10);
		gpio_reset(GPIOA, 11);
	}
	sys_param.sleep_tick_count = 0;
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

void DMA1_Channel2_CallBack(dma_event_t event, void *Parameter){
	if(event == DMA_EVENT_TRANFER_COMPLETE)
		dma_flash_rx_flag = 1;
}

void DMA1_Channel3_CallBack(dma_event_t event, void *Parameter){
	if(event == DMA_EVENT_TRANFER_COMPLETE)
		dma_flash_tx_flag = 1;
}


void read_storage_param(system_param_t *parameter){
	char *Rxbuf = (char *)malloc(40*sizeof(uint8_t));
	flash.ReadBytes(FLASH_PARAM_STORAGE_ADDRESS, (uint8_t *)Rxbuf, 40);

	/** System */
	parameter->control_type  = Rxbuf[0];
	LOG_WARN(TAG, "control_type = %d", parameter->control_type);
	parameter->enable_buzzer = Rxbuf[1];
	LOG_WARN(TAG, "enable_buzzer = %d", parameter->enable_buzzer);
	parameter->gui_theme     = Rxbuf[2];
	LOG_WARN(TAG, "gui_theme = %d", parameter->gui_theme);
	/** Sleep */
	parameter->sleep_temp_set  = (Rxbuf[3]<<8) | Rxbuf[4];
	LOG_WARN(TAG, "sleep_temp_set = %d", parameter->sleep_temp_set);
	parameter->sleep_wait_time = (Rxbuf[5]<<8) | Rxbuf[6];
	LOG_WARN(TAG, "sleep_wait_time = %d", parameter->sleep_wait_time);
	parameter->enable_weakup = Rxbuf[7];
	LOG_WARN(TAG, "enable_weakup = %d", parameter->enable_weakup);
	/** Calibration */
	parameter->temp_offset   = (Rxbuf[8]<<8) | Rxbuf[9];
	LOG_WARN(TAG, "temp_offset = %d", parameter->temp_offset);
	parameter->temp_max      = (Rxbuf[10]<<8) | Rxbuf[11];
	LOG_WARN(TAG, "temp_max = %d", parameter->temp_max);
	parameter->temp_min      = (Rxbuf[12]<<8) | Rxbuf[13];
	LOG_WARN(TAG, "temp_min = %d", parameter->temp_min);
	parameter->adc_max       = (Rxbuf[14]<<8) | Rxbuf[15];
	LOG_WARN(TAG, "adc_max = %d", parameter->adc_max);
	parameter->adc_min       = (Rxbuf[16]<<8) | Rxbuf[17];
	LOG_WARN(TAG, "adc_min = %d", parameter->adc_min);
	/** Parameter */
	parameter->encoder_step  = Rxbuf[18];
	LOG_WARN(TAG, "encoder_step = %d", parameter->encoder_step);
//	parameter->temp_error     = (Rxbuf[19]<<8) | Rxbuf[20];
	LOG_WARN(TAG, "temp_error = %d", parameter->temp_error);
	parameter->over_temp     = (Rxbuf[21]<<8) | Rxbuf[22];
	LOG_WARN(TAG, "over_temp = %d", parameter->over_temp);
	parameter->voltage_threshold = (float)Rxbuf[23] + (float)Rxbuf[24]/100.0;
	LOG_WARN(TAG, "voltage_threshold = %f", parameter->voltage_threshold);
	/** PID */
	parameter->sd_pid_param.direction = (pid_direction_t)Rxbuf[25];
	LOG_WARN(TAG, "sd_pid_param.direction = %d", parameter->sd_pid_param.direction);
	parameter->sd_pid_param.kp = (float)Rxbuf[26] + (float)Rxbuf[27]/100.0;
	LOG_WARN(TAG, "sd_pid_param.kp = %f", parameter->sd_pid_param.kp);
	parameter->sd_pid_param.ki = (float)Rxbuf[28] + (float)Rxbuf[29]/100.0;
	LOG_WARN(TAG, "sd_pid_param.ki = %f", parameter->sd_pid_param.ki);
	parameter->sd_pid_param.kd = (float)Rxbuf[30] + (float)Rxbuf[31]/100.0;
	LOG_WARN(TAG, "sd_pid_param.kd = %f", parameter->sd_pid_param.kd);
	parameter->sd_pid_param.max_output = (Rxbuf[32]<<8)| Rxbuf[33];
	LOG_WARN(TAG, "sd_pid_param.max_output = %d", parameter->sd_pid_param.max_output);
	parameter->sd_pid_param.min_output = (Rxbuf[34]<<8)| Rxbuf[35];
	LOG_WARN(TAG, "sd_pid_param.min_output = %d", parameter->sd_pid_param.min_output);
	parameter->sd_pid_param.sample_time = (Rxbuf[36]<<8) | Rxbuf[37];
	LOG_WARN(TAG, "sd_pid_param.sample_time = %d", parameter->sd_pid_param.sample_time);

	parameter->temp_set = (Rxbuf[38]<<8) | Rxbuf[39];
	LOG_WARN(TAG, "temp_set = %d", parameter->temp_set);

	free(Rxbuf);
}

void write_storage_param(system_param_t *parameter){
	char *Data = (char *)malloc(40*sizeof(uint8_t));

	/** System */
	Data[0] = parameter->control_type;
	Data[1] = parameter->enable_buzzer;
	Data[2] = parameter->gui_theme;
	/** Sleep */
	Data[3] = (parameter->sleep_temp_set >> 8);
	Data[4] = (parameter->sleep_temp_set & 0xFF);
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
	Data[19] = (parameter->temp_error >> 8);
	Data[20] = (parameter->temp_error & 0xFF);
	Data[21] = (parameter->over_temp >> 8);
	Data[22] = (parameter->over_temp & 0xFF);
	Data[23] = (uint8_t)(parameter->voltage_threshold);
	Data[24] = (uint8_t)((float)(parameter->voltage_threshold - (uint8_t)parameter->voltage_threshold))*100;
	/** PID */
	Data[25] = parameter->sd_pid_param.direction;
	Data[26] = (uint8_t)(parameter->sd_pid_param.kp);
	Data[27] = (uint8_t)(parameter->sd_pid_param.kp - Data[26])*100;
	Data[28] = (uint8_t)(parameter->sd_pid_param.ki);
	Data[29] = (uint8_t)(parameter->sd_pid_param.ki - Data[28])*100;
	Data[30] = (uint8_t)(parameter->sd_pid_param.kd);
	Data[31] = (uint8_t)(parameter->sd_pid_param.kd - Data[30])*100;
	Data[32] = (uint8_t)(parameter->sd_pid_param.max_output >> 8);
	Data[33] = (uint8_t)(parameter->sd_pid_param.max_output & 0xFF);
	Data[34] = (uint8_t)(parameter->sd_pid_param.min_output >> 8);
	Data[35] = (uint8_t)(parameter->sd_pid_param.min_output & 0xFF);
	Data[36] = (uint8_t)(parameter->sd_pid_param.sample_time >> 8);
	Data[37] = (uint8_t)(parameter->sd_pid_param.sample_time & 0xFF);

	Data[38] = (parameter->temp_set >> 8);
	Data[39] = (parameter->temp_set & 0xFF);

	flash.EraseSector(FLASH_PARAM_STORAGE_ADDRESS);
	flash.WriteBytes(FLASH_PARAM_STORAGE_ADDRESS, (uint8_t *)Data, 40);

	free(Data);
}

void beep(uint8_t loop){
	for(uint8_t i=0; i<loop; i++){
		gpio_set(GPIOA, 11);
		vTaskDelay(80);
		gpio_reset(GPIOA, 11);
		vTaskDelay(80);
	}
}

