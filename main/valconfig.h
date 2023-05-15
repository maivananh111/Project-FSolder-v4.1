/*
 * valconfig.h
 *
 *  Created on: May 10, 2023
 *      Author: anh
 */

#ifndef VALCONFIG_H_
#define VALCONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "periph/dma.h"
#include "periph/spi.h"
#include "periph/tim.h"
#include "periph/adc.h"
#include "button/button.h"
#include "gui/gui.h"



/**
 * SPI2 and DMA1_Channel5 for TFT LCD.
 */
#define TFT_SPI spi2
#define TFT_DMA dma1_channel5
dma_config_t dma1_channel5_conf = {
	.channel = DMA_Channel5,
	.direction = DMA_MEM_TO_PERIPH,
	.mode = DMA_MODE_CIRCULAR,
	.datasize = DMA_DATASIZE_8BIT,
	.interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.channelpriority = DMA_CHANNEL_PRIORITY_VERYHIGH,
	.interruptpriority = 5,
};

spi_config_t spi2_conf = {
	.mode = SPI_HALFDUPLEX_MASTER,
	.control = SPI_DMA_CONTROL,
	.clockdivision = SPI_CLOCKDIVISION_2,
	.clkport = GPIOB,
	.clkpin = 13,
	.mosiport = GPIOB,
	.mosipin = 15,
	.txdma = dma1_channel5,
};

/**
 * TIM1 for encoder.
 */
#define ENC_TIM tim1
tim_config_t tim_enc_base_conf = {
	.prescaler = 1,
	.reload = 0xFFFF,
	.interrupt = TIM_INTERRUPT_ENABLE,
	.interruptpriority = 6,
};
tim_encoder_t tim_enc_conf = {
	.mode = TIM_ENCODER_MODE3,
	.encA_ch1_port = GPIOA,
	.encA_ch1_pin = 8,
	.encB_ch2_port = GPIOA,
	.encB_ch2_pin = 9,
	.encA_ch1_edge = TIM_FALLING_EDGE,
	.encB_ch2_edge = TIM_FALLING_EDGE,
};

/**
 * TIM2 channel2 for hot air heat wire PWM.
 */
#define HA_TIM tim2
tim_config_t tim_ha_base_conf = {
	.prescaler = 72,
	.reload = 1000,
};
tim_pwm_t tim_ha_pwm_conf = {
	.port = GPIOB,
	.pin = 3,
	.preload = TIM_PRELOAD_ENABLE,
};

/**
 * TIM3 channel 1 for hot air heat wire PWM.
 */
#define SD_TIM tim3
tim_config_t tim_sd_base_conf = {
	.prescaler = 72,
	.reload = 1000,
};
tim_pwm_t tim_sd_pwm_conf = {
	.port = GPIOB,
	.pin = 4,
	.preload = TIM_PRELOAD_ENABLE,
};

/**
 * ADC1 dma DMA1_Channel1.
 * ADC solder temperature pin A0.
 * ADC solder current pin A1.
 * ADC hotair temperature pin A3.
 * ADC hotair current pin A4.
 * ADC input voltage pin B0.
 * ADC inside temperature pin B1.
 */
#define ADC_ADC adc1
#define ADC_DMA dma1_channel1
dma_config_t dma1_channel1_conf = {
	.channel = DMA_Channel1,
	.direction = DMA_PERIH_TO_MEM,
	.mode = DMA_MODE_CIRCULAR,
	.datasize = DMA_DATASIZE_16BIT,
	.interruptoption = DMA_TRANSFER_ERROR_INTERRUPT,
	.channelpriority = DMA_CHANNEL_PRIORITY_MEDIUM,
	.interruptpriority = 6,
};
GPIO_TypeDef *port_list[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB};
uint16_t pin_list[] = {0, 1, 3, 4, 0, 1};
uint8_t rank_list[] = {0, 1, 3, 4, 8, 9};
adc_sampletime_t sampletime_list[] = {ADC_28_5_CYCLES, ADC_28_5_CYCLES, ADC_28_5_CYCLES, ADC_28_5_CYCLES, ADC_28_5_CYCLES, ADC_28_5_CYCLES};
adc_config_t adc1_conf = {
	.prescaler = ADC_PRESCALER_6,
	.continuos = ADC_CONTINUOS_ENABLE,
	.port_list = port_list,
	.pin_list = pin_list,
	.rank_list = rank_list,
	.sampletime_list = sampletime_list,
	.num_channel = 6,
	.dma = dma1_channel1,
};








/**
 * Encoder button at PA10.
 */
extern void enc_button_event_handler(button_event_t);
button_t enc_button = {
	.port = GPIOA,
	.pin = 10,
	.trigger_type = BUTTON_TRIGGER_INTERRUPT,
	.interruptpriority = 5,
	.event_option = BUTTON_FULLOPTION,
	.longpress_time = 300U,
	.doubleclick_release_time = 150U,
	.bits = 3,
	.event_handler = enc_button_event_handler,
};


parameter_t show_data = {
	.tempset = 425,
	.temp = 375,
	.voltage = 23.57,
	.power = 51.3,
	.intemp = 40,
	.time = {
		.seconds = 0,
		.minutes = 0,
		.hour = 0,
		.dayofweek = 0,
		.dayofmonth = 0,
		.month = 0,
		.year = 0,
	},
	.unit = true,
};

uint16_t sd_temp_set = 0; // PID setpoint.
pid_param_t sd_pid_param = {
	.kp = 0.0,
	.ki = 0.0,
	.kd = 0.0,
	.max_output = 0,
	.min_output = 0,
	.sample_time = 100U, // 10ms.
};
pid sd_pid;

#ifdef __cplusplus
}
#endif

#endif /* VALCONFIG_H_ */
